#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
  #include <hardware/watchdog.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <RTClib.h>
#include <target.h>

/* ---------------------------------- SERIAL CONFIGURATION ---------------------------------- */

// Hardware Serial support (for UART communication on RX/TX pins)
#ifdef USE_HARDWARE_SERIAL
  #if defined(ESP32)
    #ifndef UART_RX_PIN
      #error "UART_RX_PIN must be defined in platformio.ini"
    #endif
    #ifndef UART_TX_PIN
      #error "UART_TX_PIN must be defined in platformio.ini"
    #endif
    #ifndef UART_BAUD
      #define UART_BAUD 115200
    #endif
    
    // Use Serial1 for hardware UART
    #define SerialPort Serial1
    
    void initSerial() {
      Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    }
  #else
    #define SerialPort Serial
    void initSerial() {
      SerialPort.begin(115200);
    }
  #endif
#else
  // Default: USB Serial
  #define SerialPort Serial
  void initSerial() {
    SerialPort.begin(115200);
  }
#endif

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT   "v2.1 (build: 1 Jan 2026)"

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     250
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         100
#endif

#include <helpers/BaseChatMesh.h>

// Timeout constants for message acknowledgments
#define SEND_TIMEOUT_BASE_MILLIS          500    // Base timeout for any message
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f  // Multiplier for flood routing (allows multiple hops)
#define DIRECT_SEND_PERHOP_FACTOR         6.0f   // Multiplier per hop for direct routing
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250    // Extra delay per hop to account for processing

// Demo public channel PSK (base64 encoded) - This is PUBLIC, use for testing only!
#define  PUBLIC_GROUP_PSK  "izOH6cXN6mrJ5e26oRXNcg=="

// Contact file format version for future compatibility
#define CONTACT_FILE_VERSION  1

// Key generation timeout
#define KEY_GEN_TIMEOUT_MS  30000

// Maximum reasonable clock value (year ~2100)
#define MAX_REASONABLE_TIMESTAMP  4102444800UL

// Message history
#define MAX_MESSAGE_HISTORY  10

// ANSI color codes (now runtime configurable)
#define ANSI_RESET      "\033[0m"
#define ANSI_BOLD       "\033[1m"
#define ANSI_DIM        "\033[2m"
#define ANSI_RED        "\033[31m"
#define ANSI_GREEN      "\033[32m"
#define ANSI_YELLOW     "\033[33m"
#define ANSI_BLUE       "\033[34m"
#define ANSI_MAGENTA    "\033[35m"
#define ANSI_CYAN       "\033[36m"
#define ANSI_BELL       "\007"

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

/* -------------------------------------------------------------------------------------- */

struct MessageHistoryEntry {
  char from_name[32];
  char text[MAX_TEXT_LEN + 1];
  uint32_t timestamp;
  uint8_t routing_type; // 0=direct, 1=flood, 2=public
};

struct NodePrefs {  // persisted to file
  float airtime_factor;
  char node_name[32];
  double node_lat, node_lon;
  float freq;
  uint8_t tx_power_dbm;
  float bw;
  uint8_t sf;
  uint8_t cr;
  uint8_t use_ansi_colors;  // 0 = off, 1 = on
};

// Retry configuration
#define MAX_SEND_ATTEMPTS  3      // Try 3 times before giving up
#define RETRY_FALLBACK_ATTEMPT  2  // Fall back to flood on attempt 2+

// Deduplication configuration
#define RECENT_MSG_CACHE_SIZE  10  // Track last 10 received messages for dedup

struct RecentMessageEntry {
  uint32_t hash;  // Hash of (timestamp + sender_pub_key + text)
  uint32_t recv_time;  // When we received it (for aging out)
};

// Maximum text that can be safely sent (accounting for protocol overhead)
#define SAFE_TEXT_LEN  (MAX_TEXT_LEN - 5)

class MyMesh : public BaseChatMesh, ContactVisitor {
  FILESYSTEM* _fs;
  NodePrefs _prefs;
  uint32_t expected_ack_crc;
  ChannelDetails* _public;
  unsigned long last_msg_sent;
  ContactInfo* curr_recipient;
  char command[512+10];
  uint8_t tmp_buf[256];
  char hex_buf[512];
  MessageHistoryEntry message_history[MAX_MESSAGE_HISTORY];
  uint8_t history_count;
  uint8_t history_index;
  
  // Retry state
  char pending_message[MAX_TEXT_LEN + 1];
  uint8_t send_attempt;
  uint32_t pending_timestamp;
  
  // Deduplication state
  RecentMessageEntry recent_messages[RECENT_MSG_CACHE_SIZE];
  uint8_t recent_msg_index;
  
  // Track if last command failed due to message being too long
  bool last_send_too_long;
  
  // Radio state tracking
  float last_snr;
  
  // Time sync consensus tracking
  #define TIME_SAMPLE_SIZE 5
  uint32_t time_samples[TIME_SAMPLE_SIZE];
  uint8_t time_sample_count;

  // ============================================================================
  // HELPER METHODS
  // ============================================================================
  
  // Return ANSI code if enabled, empty string otherwise
  const char* ansi(const char* code) const {
    return _prefs.use_ansi_colors ? code : "";
  }

  const char* getTypeName(uint8_t type) const {
    if (type == ADV_TYPE_CHAT) return "Chat";
    if (type == ADV_TYPE_REPEATER) return "Repeater";
    if (type == ADV_TYPE_ROOM) return "Room";
    return "??";  // unknown
  }

  File openFileForRead(const char* path) {
  #if defined(RP2040_PLATFORM)
    return _fs->open(path, "r");
  #else
    return _fs->open(path);
  #endif
  }

  File openFileForWrite(const char* path) {
  #if defined(NRF52_PLATFORM)
    _fs->remove(path);
    return _fs->open(path, FILE_O_WRITE);
  #elif defined(RP2040_PLATFORM)
    return _fs->open(path, "w");
    #else
    return _fs->open(path, "w", true);
    #endif
  }

  bool checkFilesystem() {
    if (!_fs) {
      SerialPort.println("ERROR: Filesystem not initialized");
      return false;
    }
    return true;
  }

  void printError(const char* msg) {
    SerialPort.print("   ERROR: ");
    SerialPort.println(msg);
  }

  void printSuccess(const char* msg) {
    SerialPort.print("   ");
    SerialPort.println(msg);
  }

  void printInfo(const char* msg) {
    SerialPort.print("   ");
    SerialPort.println(msg);
  }

  void showPrompt() {
    if (curr_recipient) {
      SerialPort.printf("%s[%s]%s > ", 
                    ansi(ANSI_BOLD), curr_recipient->name, ansi(ANSI_RESET));
    } else {
      SerialPort.print("[no recipient] > ");
    }
  }

  void clearCurrentLine() {
    if (_prefs.use_ansi_colors) {
      SerialPort.print("\r\033[K");  // Move to start of line and clear it (ANSI)
    } else {
      SerialPort.print("\r");  // Just move to start of line
    }
  }

  void updateTerminalTitle() {
    if (_prefs.use_ansi_colors) {
      SerialPort.print("\033]0;MeshChat");
      if (curr_recipient) {
        SerialPort.printf(" - %s", curr_recipient->name);
      }
      SerialPort.print("\007");  // Bell character ends the sequence
    }
  }

  void ringBell() {
    if (_prefs.use_ansi_colors) {
      SerialPort.print(ANSI_BELL);
    }
  }

  void showSignalBar(float snr) {
    // Convert SNR to 0-5 bars (typical range -20 to +10 dB)
    int bars = 0;
    if (snr >= -15) bars = 1;
    if (snr >= -10) bars = 2;
    if (snr >= -5) bars = 3;
    if (snr >= 0) bars = 4;
    if (snr >= 5) bars = 5;
    
    SerialPort.print("[");
    for (int i = 0; i < 5; i++) {
      if (i < bars) {
        SerialPort.print(ansi(ANSI_GREEN));
        SerialPort.print("#");
      } else {
        SerialPort.print(ansi(ANSI_DIM));
        SerialPort.print("-");
      }
    }
    SerialPort.print(ansi(ANSI_RESET));
    SerialPort.print("]");
  }

  void makeHyperlink(const char* url, const char* text) {
    if (_prefs.use_ansi_colors) {
      SerialPort.printf("\033]8;;%s\033\\%s\033]8;;\033\\", url, text);
    } else {
      SerialPort.print(text);
    }
  }

  void showPromptWithBuffer() {
    clearCurrentLine();  // Clear the current prompt line
    showPrompt();
    // Re-display any text that was being typed
    if (command[0] != 0) {
      SerialPort.print(command);
    }
  }

  bool validateTextLength(const char* text, size_t max_len) {
    size_t len = strlen(text);
    if (len == 0) {
      printError("Message text is empty");
      last_send_too_long = false;
      return false;
    }
    if (len > max_len) {
      SerialPort.printf("%s   ERROR: Message too long (%d/%d chars)%s\n", 
                    ansi(ANSI_RED), len, max_len, ansi(ANSI_RESET));
      SerialPort.printf("%s   Tip: Remove %d characters and press Enter to retry%s\n",
                    ansi(ANSI_YELLOW), len - max_len, ansi(ANSI_RESET));
      last_send_too_long = true;
      return false;
    }
    last_send_too_long = false;
    return true;
  }

  bool validateName(const char* name, size_t max_len = 31) {
    size_t len = strlen(name);
    if (len == 0 || len > max_len) {
      SerialPort.printf("   ERROR: Name must be 1-%d characters\n", max_len);
      return false;
    }
    return true;
  }

  bool requireRecipient() {
    if (!curr_recipient) {
      printError("No recipient selected");
      return false;
    }
    return true;
  }
  
  // Helper to resolve contact by name or index number
  ContactInfo* resolveContact(const char* name_or_index) {
    // Check if input is a number (contact index)
    bool is_number = true;
    for (const char* p = name_or_index; *p; p++) {
      if (*p < '0' || *p > '9') {
        is_number = false;
        break;
      }
    }
    
    if (is_number) {
      int index = atoi(name_or_index) - 1;  // Convert to 0-based index
      
      // Build sorted index array (minimal stack usage)
      struct ContactEntry {
        uint8_t idx;
        uint32_t timestamp;
      };
      ContactEntry entries[MAX_CONTACTS];
      int contact_count = 0;
      
      for (uint32_t i = 0; i < MAX_CONTACTS && contact_count < MAX_CONTACTS; i++) {
        ContactInfo c;
        if (getContactByIdx(i, c)) {
          entries[contact_count].idx = i;
          entries[contact_count].timestamp = c.last_advert_timestamp;
          contact_count++;
        }
      }
      
      // Sort by last seen (most recent first)
      for (int i = 0; i < contact_count - 1; i++) {
        for (int j = i + 1; j < contact_count; j++) {
          if (entries[j].timestamp > entries[i].timestamp) {
            ContactEntry temp = entries[i];
            entries[i] = entries[j];
            entries[j] = temp;
          }
        }
      }
      
      if (index >= 0 && index < contact_count) {
        // Get the contact at the sorted position
        ContactInfo target;
        if (getContactByIdx(entries[index].idx, target)) {
          // Find in contact table by name (to get the actual pointer)
          return searchContactsByPrefix(target.name);
        }
      }
      return NULL;
    }
    
    // Try name/prefix search
    return searchContactsByPrefix(name_or_index);
  }
  
  bool isRecentMessage(uint32_t timestamp, const uint8_t* sender_pub_key, const char* text) {
    // Calculate hash of this message
    uint32_t msg_hash;
    uint8_t temp_buf[4 + PUB_KEY_SIZE + MAX_TEXT_LEN + 1];
    int len = 0;
    
    memcpy(&temp_buf[len], &timestamp, 4); len += 4;
    memcpy(&temp_buf[len], sender_pub_key, PUB_KEY_SIZE); len += PUB_KEY_SIZE;
    int text_len = strlen(text);
    memcpy(&temp_buf[len], text, text_len); len += text_len;
    
    mesh::Utils::sha256((uint8_t*)&msg_hash, 4, temp_buf, len);
    
    // Check if we've seen this hash recently
    uint32_t now = getRTCClock()->getCurrentTime();
    for (int i = 0; i < RECENT_MSG_CACHE_SIZE; i++) {
      if (recent_messages[i].hash == msg_hash) {
        // Found duplicate - check if it's not too old (within 5 minutes)
        if (now - recent_messages[i].recv_time < 300) {
          return true;  // It's a recent duplicate
        }
      }
    }
    
    // Not a duplicate - add to cache
    recent_messages[recent_msg_index].hash = msg_hash;
    recent_messages[recent_msg_index].recv_time = now;
    recent_msg_index = (recent_msg_index + 1) % RECENT_MSG_CACHE_SIZE;
    
    return false;
  }
  
  void trySendPendingMessage() {
    if (!curr_recipient || pending_message[0] == 0) return;
    
    if (send_attempt == 0) {
      clearCurrentLine();
      SerialPort.printf("%s-- Sending...%s\n", ansi(ANSI_DIM), ansi(ANSI_RESET));
    }
    
    uint32_t est_timeout;
    int result = sendMessage(*curr_recipient, pending_timestamp, send_attempt, pending_message, expected_ack_crc, est_timeout);
    
    if (result == MSG_SEND_FAILED) {
      printError("Send failed");
      pending_message[0] = 0;  // Clear pending
    } else {
      last_msg_sent = _ms->getMillis();
      if (send_attempt == 0) {
        // Only show confirmation on first send, not retries
        // Add sent message to history
        addMessageToHistory(_prefs.node_name, pending_message, pending_timestamp, 
                           result == MSG_SEND_SENT_FLOOD ? 1 : 0);
      }
    }
    
    if (send_attempt == 0) {
      showPromptWithBuffer();
    }
  }

  // ============================================================================
  // COMMAND HANDLERS
  // ============================================================================

  void cmdSend(const char* text) {
    if (!curr_recipient) {
      printError("no recipient selected (use 'to <name>' or 'to public').");
      last_send_too_long = false;
      return;
    }

    // Check if sending to public channel - need to account for "Name: " prefix
    if (strcmp(curr_recipient->name, "Public") == 0) {
      // For public: "NodeName: message" format
      int prefix_len = strlen(_prefs.node_name) + 2;  // +2 for ": "
      int max_msg_len = SAFE_TEXT_LEN - prefix_len;
      if (!validateTextLength(text, max_msg_len)) return;
      cmdPublic(text);
      return;
    }
    
    // For direct messages, use safe text length with overhead margin
    if (!validateTextLength(text, SAFE_TEXT_LEN)) return;

    // Store message for potential retries
    StrHelper::strncpy(pending_message, text, sizeof(pending_message));
    pending_timestamp = getRTCClock()->getCurrentTime();
    send_attempt = 0;
    
    // Attempt to send
    trySendPendingMessage();
  }

  void cmdPublic(const char* msg_text) {
    if (!_public) {
      printError("Public channel not available");
      last_send_too_long = false;
      return;
    }
    // Validation already done in cmdSend with proper prefix accounting

    uint8_t temp[5+MAX_TEXT_LEN+32];
    uint32_t timestamp = getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4);
    temp[4] = 0;

    int msg_len = snprintf((char *) &temp[5], MAX_TEXT_LEN, "%s: %s", _prefs.node_name, msg_text);
    if (msg_len >= MAX_TEXT_LEN) {
      // This should never happen now due to validation, but keep as safety
      SerialPort.printf("%s   Message too long (truncated)%s\n", ansi(ANSI_YELLOW), ansi(ANSI_RESET));
      temp[5 + MAX_TEXT_LEN - 1] = 0;
      msg_len = MAX_TEXT_LEN - 1;
    }

    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, _public->channel, temp, 5 + msg_len);
    if (pkt) {
      sendFlood(pkt);
      clearCurrentLine();
      SerialPort.printf("%s-- Sent to public%s\n", ansi(ANSI_DIM), ansi(ANSI_RESET));
      
      // Add sent public message to history
      addMessageToHistory(_prefs.node_name, msg_text, timestamp, 2);
      showPromptWithBuffer();
    } else {
      printError("Send failed");
    }
  }

  void cmdList(int n) {
    SerialPort.println();
    
    // Build array of contact indices with timestamps (minimal stack usage)
    struct ContactEntry {
      uint8_t idx;
      uint32_t timestamp;
    };
    ContactEntry entries[MAX_CONTACTS];
    int contact_count = 0;
    
    // Collect contact indices and timestamps
    for (uint32_t i = 0; i < MAX_CONTACTS && contact_count < MAX_CONTACTS; i++) {
      ContactInfo c;
      if (getContactByIdx(i, c)) {
        entries[contact_count].idx = i;
        entries[contact_count].timestamp = c.last_advert_timestamp;
        contact_count++;
      }
    }
    
    if (contact_count == 0) {
      SerialPort.println("No contacts");
      return;
    }
    
    // Sort by last seen (most recent first)
    for (int i = 0; i < contact_count - 1; i++) {
      for (int j = i + 1; j < contact_count; j++) {
        if (entries[j].timestamp > entries[i].timestamp) {
          ContactEntry temp = entries[i];
          entries[i] = entries[j];
          entries[j] = temp;
        }
      }
    }
    
    // Display contacts with numbers
    int display_count = (n > 0 && n < contact_count) ? n : contact_count;
    for (int i = 0; i < display_count; i++) {
      ContactInfo c;
      if (getContactByIdx(entries[i].idx, c)) {
        char tmp[40];
        int32_t secs = c.last_advert_timestamp - getRTCClock()->getCurrentTime();
        AdvertTimeHelper::formatRelativeTimeDiff(tmp, secs, false);
        SerialPort.printf("   %s[%d]%s %s - %s\n", 
                      ansi(ANSI_DIM), i + 1, ansi(ANSI_RESET),
                      c.name, tmp);
      }
    }
  }

  void cmdHistory() {
    SerialPort.println();
    if (history_count == 0) {
      SerialPort.println("No messages");
      return;
    }

    SerialPort.printf("%sMessage History:%s\n", ansi(ANSI_BOLD), ansi(ANSI_RESET));
    int start = history_index >= history_count ? 0 : history_index;
    for (int i = 0; i < history_count; i++) {
      int idx = (start + i) % MAX_MESSAGE_HISTORY;
      MessageHistoryEntry& entry = message_history[idx];
      
      // Format timestamp
      char time_str[20];
      DateTime dt = DateTime(entry.timestamp);
      snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
      
      // Match real-time display format
      if (entry.routing_type == 2) {
        // Public channel: * username: message
        SerialPort.printf("[%s] %s*%s %s\n", 
                      time_str, ansi(ANSI_MAGENTA), ansi(ANSI_RESET), entry.text);
      } else {
        // Direct message: <username> message
        SerialPort.printf("[%s] %s<%s>%s %s\n", 
                      time_str, ansi(ANSI_CYAN), entry.from_name, ansi(ANSI_RESET), entry.text);
      }
    }
  }

  void cmdClock() {
    uint32_t now = getRTCClock()->getCurrentTime();
    DateTime dt = DateTime(now);
    SerialPort.printf("   %02d:%02d:%02d %d/%02d/%d (epoch %lu)\n", 
                  dt.hour(), dt.minute(), dt.second(),
                  dt.year(), dt.month(), dt.day(), now);
  }

  void cmdSetTime(const char* time_str) {
    // Try to parse as epoch first
    uint32_t secs = _atoi(time_str);
    if (secs > 1600000000) {  // Valid epoch timestamp
      setClock(secs);
      return;
    }
    
    // Try to parse as date/time: "dd/mm/yyyy hh:mm" or "yyyy-mm-dd hh:mm"
    int day, month, year, hour, minute;
    
    // Try format: dd/mm/yyyy hh:mm
    if (sscanf(time_str, "%d/%d/%d %d:%d", &day, &month, &year, &hour, &minute) == 5) {
      if (year < 100) year += 2000;  // Support 2-digit years
      DateTime dt(year, month, day, hour, minute, 0);
      setClock(dt.unixtime());
      return;
    }
    
    // Try format: yyyy-mm-dd hh:mm
    if (sscanf(time_str, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute) == 5) {
      DateTime dt(year, month, day, hour, minute, 0);
      setClock(dt.unixtime());
      return;
    }
    
    // Try format: dd/mm/yyyy (assume 00:00)
    if (sscanf(time_str, "%d/%d/%d", &day, &month, &year) == 3) {
      if (year < 100) year += 2000;
      DateTime dt(year, month, day, 0, 0, 0);
      setClock(dt.unixtime());
      return;
    }
    
    printError("Invalid format. Use: dd/mm/yyyy hh:mm or epoch");
  }

  void cmdSetRecipient(const char* name) {
    if (!validateName(name, 31)) return;
    
    // Special case: "public" channel
    if (strcmp(name, "public") == 0 || strcmp(name, "Public") == 0) {
      if (!_public) {
        printError("Public channel not available");
        return;
      }
      // Create a pseudo-contact for the public channel
      static ContactInfo public_pseudo;
      memset(&public_pseudo, 0, sizeof(public_pseudo));
      StrHelper::strncpy(public_pseudo.name, "Public", sizeof(public_pseudo.name));
      public_pseudo.type = ADV_TYPE_ROOM;  // Public channel is a "room"
      curr_recipient = &public_pseudo;
      SerialPort.println("   To: Public channel");
      updateTerminalTitle();
      return;
    }
    
    // Use helper to resolve by name or index
    curr_recipient = resolveContact(name);
    if (curr_recipient) {
      SerialPort.printf("   To: %s\n", curr_recipient->name);
      updateTerminalTitle();
    } else {
      SerialPort.printf("   ERROR: '%s' not found\n", name);
    }
  }

  void cmdShowRecipient() {
    if (curr_recipient) {
      SerialPort.printf("   To: %s\n", curr_recipient->name);
    } else {
      SerialPort.println("   (none - use 'to <name>')");
    }
  }

  void cmdAdvert() {
    auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
    if (pkt) {
      sendZeroHop(pkt);
      SerialPort.println("   Advert sent");
    } else {
      printError("Send failed");
    }
  }

  void cmdResetPath() {
    if (!requireRecipient()) return;
    
    resetPathTo(*curr_recipient);
    saveContacts();
    SerialPort.println("   Path reset");
  }

  void cmdDelete(const char* name) {
    ContactInfo* contact = resolveContact(name);
    if (!contact) {
      SerialPort.printf("   ERROR: Contact '%s' not found\n", name);
      return;
    }

    if (removeContact(*contact)) {
      if (curr_recipient && curr_recipient->id.pub_key[0] == contact->id.pub_key[0]) {
        curr_recipient = NULL;  // Clear recipient if it was deleted
      }
      saveContacts();
      SerialPort.println("   Deleted");
    } else {
      printError("Failed to delete contact");
    }
  }

  void cmdRename(const char* old_name, const char* new_name) {
    if (!validateName(new_name)) return;
    
    ContactInfo* contact = resolveContact(old_name);
    if (!contact) {
      SerialPort.printf("   ERROR: Contact '%s' not found\n", old_name);
      return;
    }

    StrHelper::strncpy(contact->name, new_name, sizeof(contact->name));
    saveContacts();
    SerialPort.printf("   Renamed to %s\n", contact->name);
  }

  void cmdCard() {
    SerialPort.printf("Hello %s\n", _prefs.node_name);
    auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
    if (!pkt) {
      printError("Failed to create advert");
      return;
    }

    uint8_t len =  pkt->writeTo(tmp_buf);
    releasePacket(pkt);
    
    if (len > 0 && len <= sizeof(tmp_buf)) {
      mesh::Utils::toHex(hex_buf, tmp_buf, len);
      SerialPort.println("Your MeshCore biz card:");
      SerialPort.print("meshcore://");
      
      // Make it a clickable hyperlink if ANSI is enabled
      char full_url[600];
      snprintf(full_url, sizeof(full_url), "meshcore://%s", hex_buf);
      makeHyperlink(full_url, hex_buf);
      SerialPort.println();
      SerialPort.println();
    } else {
      printError("Card generation failed");
    }
  }

  void cmdInfo() {
    if (!requireRecipient()) return;

    SerialPort.printf("Contact: %s\n", curr_recipient->name);
    SerialPort.printf("   Type: %s\n", getTypeName(curr_recipient->type));
    SerialPort.print("   Public key: "); 
    mesh::Utils::printHex(SerialPort, curr_recipient->id.pub_key, PUB_KEY_SIZE); 
    SerialPort.println();
    SerialPort.printf("   Path length: %d hops\n", curr_recipient->out_path_len);
    if (curr_recipient->gps_lat != 0 || curr_recipient->gps_lon != 0) {
      SerialPort.printf("   GPS: %.6f, %.6f\n", curr_recipient->gps_lat, curr_recipient->gps_lon);
    }
  }

  void cmdRadio() {
    SerialPort.printf("Radio: %.2f MHz, BW: %.1f kHz\n", _prefs.freq, _prefs.bw);
    SerialPort.printf("TX: %d dBm, SF: %d, CR: %d\n", _prefs.tx_power_dbm, _prefs.sf, _prefs.cr);
    SerialPort.printf("Airtime Factor: %.2f\n", _prefs.airtime_factor);
    if (last_snr != 0) {
      SerialPort.printf("Last RX SNR: %.1f dB ", last_snr);
      showSignalBar(last_snr);
      SerialPort.println();
    }
  }

  void cmdStatus() {
    SerialPort.printf("Node: %s\n", _prefs.node_name);
    SerialPort.printf("Contacts: %d\n", getNumContacts());
    SerialPort.printf("ANSI colors: %s\n", _prefs.use_ansi_colors ? "ON" : "OFF");
    if (_prefs.node_lat != 0 || _prefs.node_lon != 0) {
      SerialPort.printf("GPS: %.6f, %.6f\n", _prefs.node_lat, _prefs.node_lon);
    }
    SerialPort.println("(Use 'radio' for RF params)");
  }

  void cmdHelp() {
    SerialPort.println();
    SerialPort.printf("%sCommands:%s\n\n", ansi(ANSI_BOLD), ansi(ANSI_RESET));
    SerialPort.printf("%sMessaging:%s\n", ansi(ANSI_CYAN), ansi(ANSI_RESET));
    SerialPort.println("  send <text>      Send message to current recipient");
    SerialPort.println();
    SerialPort.printf("%sContacts:%s\n", ansi(ANSI_CYAN), ansi(ANSI_RESET));
    SerialPort.println("  to <name|#>      Select recipient by name or number");
    SerialPort.println("  to public        Select public channel");
    SerialPort.println("  to               Show current recipient");
    SerialPort.println("  list [n]         List contacts (with numbers)");
    SerialPort.println("  contacts         Alias for list");
    SerialPort.println("  info             Show contact details");
    SerialPort.println("  delete <name>    Remove contact");
    SerialPort.println("  rename <o> <n>   Rename contact");
    SerialPort.println();
    SerialPort.printf("%sNetwork:%s\n", ansi(ANSI_CYAN), ansi(ANSI_RESET));
    SerialPort.println("  advert           Send advertisement");
    SerialPort.println("  reset path       Reset route");
    SerialPort.println("  card             Generate card");
    SerialPort.println("  import <card>    Import contact");
    SerialPort.println();
    SerialPort.printf("%sConfig:%s\n", ansi(ANSI_CYAN), ansi(ANSI_RESET));
    SerialPort.println("  set name <val>   Set node name");
    SerialPort.println("  set lat/lon <v>  Set GPS coords");
    SerialPort.println("  set time <val>   Set time (dd/mm/yyyy hh:mm or epoch)");
    SerialPort.println("  set ansi on/off  Toggle ANSI colors");
    SerialPort.println("  set freq/tx/bw/sf/cr/af  Radio params");
    SerialPort.println();
    SerialPort.printf("%sOther:%s\n", ansi(ANSI_CYAN), ansi(ANSI_RESET));
    SerialPort.println("  history          Message history");
    SerialPort.println("  clock            Show time");
    SerialPort.println("  status           System status");
    SerialPort.println("  radio            Radio status & signal");
    SerialPort.println("  reboot           Restart device");
    SerialPort.println("  ver              Version");
    SerialPort.println("  help, ?          This help");
    SerialPort.println();
  }

  template<typename T>
  bool validateRange(T value, T min, T max, const char* param_name) {
    if (value < min || value > max) {
      SerialPort.printf("   ERROR: %s must be between ", param_name);
      SerialPort.print(min);
      SerialPort.print(" and ");
      SerialPort.println(max);
      return false;
    }
    return true;
  }

  void cmdSetConfig(const char* param) {
    if (memcmp(param, "af ", 3) == 0) {
      float new_af = atof(&param[3]);  // Skip "af "
      if (!validateRange(new_af, 0.01f, 100.0f, "Airtime factor")) return;
      _prefs.airtime_factor = new_af;
      savePrefs();
      SerialPort.printf("   AF: %.2f\n", new_af);
    } else if (memcmp(param, "ansi ", 5) == 0) {
      const char* ansi_value = &param[5];  // Skip "ansi "
      if (strcmp(ansi_value, "on") == 0 || strcmp(ansi_value, "1") == 0) {
        _prefs.use_ansi_colors = 1;
        savePrefs();
        SerialPort.println(ansi(ANSI_GREEN));
        SerialPort.println("   ANSI colors: ON");
        SerialPort.print(ansi(ANSI_RESET));
      } else if (strcmp(ansi_value, "off") == 0 || strcmp(ansi_value, "0") == 0) {
        _prefs.use_ansi_colors = 0;
        savePrefs();
        SerialPort.println("   ANSI colors: OFF");
      } else {
        printError("Use 'on' or 'off'");
      }
    } else if (memcmp(param, "time ", 5) == 0) {
      cmdSetTime(&param[5]);
    } else if (memcmp(param, "name ", 5) == 0) {
      const char* value = &param[5];  // Skip "name "
      if (!validateName(value)) return;
      StrHelper::strncpy(_prefs.node_name, value, sizeof(_prefs.node_name));
      savePrefs();
      SerialPort.printf("   Name: %s\n", _prefs.node_name);
    } else if (memcmp(param, "lat ", 4) == 0) {
      double lat = atof(&param[4]);  // Skip "lat "
      if (!validateRange(lat, -90.0, 90.0, "Latitude")) return;
      _prefs.node_lat = lat;
      savePrefs();
      SerialPort.printf("   Lat: %.6f\n", lat);
    } else if (memcmp(param, "lon ", 4) == 0) {
      double lon = atof(&param[4]);  // Skip "lon "
      if (!validateRange(lon, -180.0, 180.0, "Longitude")) return;
      _prefs.node_lon = lon;
      savePrefs();
      SerialPort.printf("   Lon: %.6f\n", lon);
    } else if (memcmp(param, "tx ", 3) == 0) {
      int tx_power = atoi(&param[3]);  // Skip "tx "
      if (!validateRange(tx_power, 2, 30, "TX power")) return;
      _prefs.tx_power_dbm = tx_power;
      savePrefs();
      SerialPort.printf("   TX: %d dBm (reboot)\n", tx_power);
    } else if (memcmp(param, "freq ", 5) == 0) {
      float freq = atof(&param[5]);  // Skip "freq "
      if (!validateRange(freq, 137.0f, 1020.0f, "Frequency")) return;
      _prefs.freq = freq;
      savePrefs();
      SerialPort.printf("   Freq: %.2f MHz (reboot)\n", freq);
    } else if (memcmp(param, "bw ", 3) == 0) {
      float bw = atof(&param[3]);  // Skip "bw "
      if (!validateRange(bw, 7.8f, 500.0f, "BW")) return;
      _prefs.bw = bw;
      savePrefs();
      SerialPort.printf("   BW: %.1f kHz (reboot)\n", bw);
    } else if (memcmp(param, "sf ", 3) == 0) {
      int sf = atoi(&param[3]);  // Skip "sf "
      if (!validateRange(sf, 5, 12, "SF")) return;
      _prefs.sf = sf;
      savePrefs();
      SerialPort.printf("   SF: %d (reboot)\n", sf);
    } else if (memcmp(param, "cr ", 3) == 0) {
      int cr = atoi(&param[3]);  // Skip "cr "
      if (!validateRange(cr, 5, 8, "CR")) return;
      _prefs.cr = cr;
      savePrefs();
      SerialPort.printf("   CR: %d (reboot)\n", cr);
    } else {
      SerialPort.printf("   ERROR: Unknown config parameter: %s\n", param);
    }
  }

  void loadContacts() {
    if (!checkFilesystem()) return;
    
    if (!_fs->exists("/contacts")) return;
    
    File file = openFileForRead("/contacts");
    if (!file) {
      SerialPort.println("Warning: Could not open contacts file");
      return;
    }
    
        // Check file version (future compatibility)
        uint8_t version = 0;
        if (file.read(&version, 1) != 1) {
      SerialPort.println("Warning: Contact file corrupted (no version)");
          file.close();
          return;
        }
        
        if (version != CONTACT_FILE_VERSION) {
      SerialPort.printf("Warning: Contact file version mismatch (found %d, expected %d)\n", version, CONTACT_FILE_VERSION);
        }
        
        bool full = false;
        int loaded_count = 0;
        while (!full) {
          ContactInfo c;
          uint8_t pub_key[32];
          uint8_t unused;
          uint32_t reserved;

          bool success = (file.read(pub_key, 32) == 32);
          success = success && (file.read((uint8_t *) &c.name, 32) == 32);
          success = success && (file.read(&c.type, 1) == 1);
          success = success && (file.read(&c.flags, 1) == 1);
          success = success && (file.read(&unused, 1) == 1);
          success = success && (file.read((uint8_t *) &reserved, 4) == 4);
          success = success && (file.read((uint8_t *) &c.out_path_len, 1) == 1);
          success = success && (file.read((uint8_t *) &c.last_advert_timestamp, 4) == 4);
          success = success && (file.read(c.out_path, 64) == 64);
          success = success && (file.read((uint8_t *) &c.gps_lat, 8) == 8);
          success = success && (file.read((uint8_t *) &c.gps_lon, 8) == 8);

          if (!success) break;  // EOF or corruption

          c.id = mesh::Identity(pub_key);
          c.lastmod = 0;
          if (!addContact(c)) {
            full = true;
          } else {
            loaded_count++;
          }
        }
        file.close();
    SerialPort.printf("Loaded %d contact(s)\n", loaded_count);
  }

  void saveContacts() {
    if (!checkFilesystem()) return;
    
    File file = openFileForWrite("/contacts");
    if (!file) {
      SerialPort.println("ERROR: Could not open contacts file for writing");
      return;
    }
    
      ContactsIterator iter;
      ContactInfo c;
      uint8_t unused = 0;
      uint32_t reserved = 0;
      uint8_t version = CONTACT_FILE_VERSION;

      // Write version header
      if (file.write(&version, 1) != 1) {
      SerialPort.println("ERROR: Failed to write contacts file");
        file.close();
        return;
      }

      while (iter.hasNext(this, c)) {
        bool success = (file.write(c.id.pub_key, 32) == 32);
        success = success && (file.write((uint8_t *) &c.name, 32) == 32);
        success = success && (file.write(&c.type, 1) == 1);
        success = success && (file.write(&c.flags, 1) == 1);
        success = success && (file.write(&unused, 1) == 1);
        success = success && (file.write((uint8_t *) &reserved, 4) == 4);
        success = success && (file.write((uint8_t *) &c.out_path_len, 1) == 1);
        success = success && (file.write((uint8_t *) &c.last_advert_timestamp, 4) == 4);
        success = success && (file.write(c.out_path, 64) == 64);
        success = success && (file.write((uint8_t *) &c.gps_lat, 8) == 8);
        success = success && (file.write((uint8_t *) &c.gps_lon, 8) == 8);

        if (!success) {
        SerialPort.println("ERROR: Failed to save contact");
          break;
        }
      }
      file.close();
  }

  void setClock(uint32_t timestamp) {
    uint32_t curr = getRTCClock()->getCurrentTime();
    
    // Validate timestamp is reasonable
    if (timestamp > MAX_REASONABLE_TIMESTAMP) {
      SerialPort.println("   (ERR: timestamp too far in future, rejected)");
      return;
    }
    
    if (timestamp < 1600000000) {  // Before Sept 2020
      SerialPort.println("   (ERR: timestamp too old, rejected)");
      return;
    }
    
    if (timestamp > curr) {
      getRTCClock()->setCurrentTime(timestamp);
      SerialPort.println("   (OK - clock set!)");
    } else {
      SerialPort.println("   (ERR: clock cannot go backwards)");
    }
  }

  void importCard(const char* command) {
    while (*command == ' ') command++;   // skip leading spaces
    if (memcmp(command, "meshcore://", 11) == 0) {
      command += 11;  // skip the prefix
      char *ep = strchr(command, 0);  // find end of string
      while (ep > command) {
        ep--;
        if (mesh::Utils::isHexChar(*ep)) break;  // found tail end of card
        *ep = 0;  // remove trailing spaces and other junk
      }
      int len = strlen(command);
      if (len % 2 == 0 && len > 0) {
        len >>= 1;  // halve, for num bytes
        if (len <= sizeof(tmp_buf)) {  // Bounds check
          if (mesh::Utils::fromHex(tmp_buf, len, command)) {
            if (importContact(tmp_buf, len)) {
              SerialPort.println("   OK - Contact imported successfully");
              saveContacts();
              return;
            } else {
              SerialPort.println("   ERROR - Failed to import contact (duplicate or full?)");
              return;
            }
          }
        } else {
          SerialPort.println("   ERROR - Card data too large");
          return;
        }
      }
    }
    SerialPort.println("   ERROR - Invalid format (expected: meshcore://HEX...)");
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return _prefs.airtime_factor;
  }

  int calcRxDelay(float score, uint32_t air_time) const override {
    return 0;  // disable rxdelay
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return true;
  }

  // Auto-sync time from incoming packets using consensus
  void autoSyncTime(uint32_t sender_timestamp) {
    uint32_t our_time = getRTCClock()->getCurrentTime();
    
    // Validate timestamp is reasonable
    if (sender_timestamp < 1600000000 || sender_timestamp > MAX_REASONABLE_TIMESTAMP) {
      return;  // Reject clearly invalid timestamps
    }
    
    // Don't sync if sender's time is too far in the past (more than 1 hour behind)
    if (sender_timestamp + 3600 < our_time) {
      return;
    }
    
    // Add to time samples
    time_samples[time_sample_count % TIME_SAMPLE_SIZE] = sender_timestamp;
    time_sample_count++;
    
    // Need at least 3 samples for consensus
    if (time_sample_count < 3) {
      return;
    }
    
    // Calculate median of recent samples (more robust than average)
    uint8_t num_samples = time_sample_count < TIME_SAMPLE_SIZE ? time_sample_count : TIME_SAMPLE_SIZE;
    uint32_t sorted[TIME_SAMPLE_SIZE];
    memcpy(sorted, time_samples, num_samples * sizeof(uint32_t));
    
    // Simple bubble sort for median
    for (uint8_t i = 0; i < num_samples - 1; i++) {
      for (uint8_t j = 0; j < num_samples - i - 1; j++) {
        if (sorted[j] > sorted[j + 1]) {
          uint32_t temp = sorted[j];
          sorted[j] = sorted[j + 1];
          sorted[j + 1] = temp;
        }
      }
    }
    
    uint32_t median_time = sorted[num_samples / 2];
    
    // Only sync if median is newer than our time and difference is significant (>10 sec)
    if (median_time > our_time + 10) {
      getRTCClock()->setCurrentTime(median_time);
      clearCurrentLine();
      SerialPort.printf("%s[Time synced: +%d sec from %d samples]%s\n", 
                    ansi(ANSI_DIM), median_time - our_time, num_samples, ansi(ANSI_RESET));
      showPromptWithBuffer();
      
      // Reset sample count after syncing to get fresh samples
      time_sample_count = 0;
    }
  }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    // Auto-sync time from contact's advertisement timestamp
    autoSyncTime(contact.last_advert_timestamp);
    
    // Only keep Chat type contacts - remove Repeaters, Rooms, and Sensors
    if (contact.type != ADV_TYPE_CHAT) {
      // Contact was already added by BaseChatMesh, so we need to remove it
      removeContact(contact);
      return;  // Don't show in UI
    }
    
    clearCurrentLine();
    if (is_new) {
      SerialPort.printf("%s-- %s joined (%d hops)%s\n", 
                    ansi(ANSI_GREEN), contact.name, path_len, ansi(ANSI_RESET));
    } else {
      SerialPort.printf("%s-- %s updated (%d hops)%s\n", 
                    ansi(ANSI_DIM), contact.name, path_len, ansi(ANSI_RESET));
    }
    
    saveContacts();
    showPromptWithBuffer();  // Re-display prompt and any typed text
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    clearCurrentLine();
    SerialPort.printf("%s-- %s path updated (%d hops)%s\n", 
                  ansi(ANSI_DIM), contact.name, (int32_t) contact.out_path_len, ansi(ANSI_RESET));
    
    saveContacts();
    showPromptWithBuffer();
  }

  ContactInfo* processAck(const uint8_t *data) override {
    if (memcmp(data, &expected_ack_crc, 4) == 0) {     // got an ACK from recipient
      clearCurrentLine();
      SerialPort.printf("%s-- ACK (%d ms)%s\n", 
                    ansi(ANSI_GREEN), _ms->getMillis() - last_msg_sent, ansi(ANSI_RESET));
      
      // NOTE: the same ACK can be received multiple times!
      expected_ack_crc = 0;  // reset our expected hash, now that we have received ACK
      pending_message[0] = 0;  // Clear pending message - successfully sent!
      showPromptWithBuffer();  // Re-display prompt and any typed text
      return NULL;  // TODO: really should return ContactInfo pointer 
    }

    //uint32_t crc;
    //memcpy(&crc, data, 4);
    //MESH_DEBUG_PRINTLN("unknown ACK received: %08X (expected: %08X)", crc, expected_ack_crc);
    return NULL;
  }

  void addMessageToHistory(const char* from, const char* text, uint32_t timestamp, uint8_t type) {
    MessageHistoryEntry& entry = message_history[history_index];
    StrHelper::strncpy(entry.from_name, from, sizeof(entry.from_name));
    StrHelper::strncpy(entry.text, text, sizeof(entry.text));
    entry.timestamp = timestamp;
    entry.routing_type = type;
    
    history_index = (history_index + 1) % MAX_MESSAGE_HISTORY;
    if (history_count < MAX_MESSAGE_HISTORY) {
      history_count++;
    }
  }

  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    // Auto-sync time if sender's timestamp is newer
    autoSyncTime(sender_timestamp);
    
    // Check for duplicate before displaying
    if (isRecentMessage(sender_timestamp, from.id.pub_key, text)) {
      // Duplicate message - ACK will still be sent automatically by BaseChatMesh, but don't display
      return;
    }
    
    // Track radio quality from last received packet
    last_snr = pkt->getSNR();
    
    ringBell();  // Notify user
    clearCurrentLine();
    
    // IRC-style format with cyan username: <username> message
    SerialPort.printf("%s<%s>%s %s\n", 
                  ansi(ANSI_CYAN), from.name, ansi(ANSI_RESET), text);
    
    addMessageToHistory(from.name, text, sender_timestamp, pkt->isRouteDirect() ? 0 : 1);

    if (strcmp(text, "clock sync") == 0) {  // special text command
      setClock(sender_timestamp + 1);
    }
    
    showPromptWithBuffer();  // Re-display prompt and any typed text
  }

  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    // Auto-sync time if sender's timestamp is newer
    autoSyncTime(sender_timestamp);
    
    // Track radio quality from last received packet
    last_snr = pkt->getSNR();
    
    clearCurrentLine();
    SerialPort.printf("!%s %s\n", from.name, text);
    showPromptWithBuffer();
  }
  
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override {
    // Auto-sync time if sender's timestamp is newer
    autoSyncTime(sender_timestamp);
    
    // Check for duplicate before displaying
    if (isRecentMessage(sender_timestamp, from.id.pub_key, text)) {
      // Duplicate message - ACK will still be sent automatically by BaseChatMesh, but don't display
      return;
    }
    
    // Track radio quality from last received packet
    last_snr = pkt->getSNR();
    
    clearCurrentLine();
    SerialPort.printf("+%s %s\n", from.name, text);
    showPromptWithBuffer();
  }

  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char *text) override {
    // Auto-sync time if sender's timestamp is newer
    autoSyncTime(timestamp);
    
    // For public channel messages, use channel hash as "sender" for dedup
    // (since multiple people can send, we use timestamp + channel + text)
    uint32_t msg_hash;
    uint8_t temp_buf[4 + 2 + MAX_TEXT_LEN + 1];
    int len = 0;
    
    memcpy(&temp_buf[len], &timestamp, 4); len += 4;
    memcpy(&temp_buf[len], channel.hash, 2); len += 2;  // Use channel hash
    int text_len = strlen(text);
    memcpy(&temp_buf[len], text, text_len); len += text_len;
    
    mesh::Utils::sha256((uint8_t*)&msg_hash, 4, temp_buf, len);
    
    // Check recent cache
    uint32_t now = getRTCClock()->getCurrentTime();
    for (int i = 0; i < RECENT_MSG_CACHE_SIZE; i++) {
      if (recent_messages[i].hash == msg_hash && now - recent_messages[i].recv_time < 300) {
        return;  // Duplicate public message
      }
    }
    
    // Not duplicate - add to cache
    recent_messages[recent_msg_index].hash = msg_hash;
    recent_messages[recent_msg_index].recv_time = now;
    recent_msg_index = (recent_msg_index + 1) % RECENT_MSG_CACHE_SIZE;
    
    // Track radio quality from last received packet
    last_snr = pkt->getSNR();
    
    ringBell();  // Notify user
    clearCurrentLine();
    
    // IRC-style format for public with magenta asterisk: * username: message
    SerialPort.printf("%s*%s %s\n", 
                  ansi(ANSI_MAGENTA), ansi(ANSI_RESET), text);
    
    addMessageToHistory("Public", text, timestamp, 2);
    showPromptWithBuffer();  // Re-display prompt and any typed text
  }

  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override {
    return 0;  // unknown
  }

  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {
    // not supported
  }

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS + 
         ( (pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_len + 1));
  }

  void onSendTimeout() override {
    // Only show timeout if we haven't already received the ACK
    if (expected_ack_crc != 0) {
      clearCurrentLine();
      
      // Try to retry if we have attempts left
      if (send_attempt < MAX_SEND_ATTEMPTS && pending_message[0] != 0) {
        send_attempt++;
        
        // Fall back to flood routing after failed direct attempts
        if (send_attempt >= RETRY_FALLBACK_ATTEMPT && curr_recipient && curr_recipient->out_path_len >= 0) {
          SerialPort.printf("%s-- Timeout, retrying via flood...%s\n", ansi(ANSI_YELLOW), ansi(ANSI_RESET));
          curr_recipient->out_path_len = -1;  // Force flood
        } else {
          SerialPort.printf("%s-- Timeout, retry %d/%d...%s\n", 
                        ansi(ANSI_YELLOW), send_attempt, MAX_SEND_ATTEMPTS, ansi(ANSI_RESET));
        }
        
        trySendPendingMessage();
      } else {
        // All retries exhausted
        SerialPort.printf("%s-- Send failed (no ACK)%s\n", ansi(ANSI_RED), ansi(ANSI_RESET));
        pending_message[0] = 0;  // Clear pending message
        expected_ack_crc = 0;
      }
      
      showPromptWithBuffer();
    }
  }

public:
  MyMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    // defaults
    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.airtime_factor = 1.0;    // 1/2 duty cycle (same as companion_radio)
    strcpy(_prefs.node_name, "NONAME");
    _prefs.freq = LORA_FREQ;
    _prefs.tx_power_dbm = LORA_TX_POWER;
    _prefs.bw = LORA_BW;
    _prefs.sf = LORA_SF;
    _prefs.cr = LORA_CR;
    _prefs.use_ansi_colors = 0;  // Default: ANSI colors disabled

    command[0] = 0;
    curr_recipient = NULL;
    history_count = 0;
    history_index = 0;
    pending_message[0] = 0;
    send_attempt = 0;
    pending_timestamp = 0;
    recent_msg_index = 0;
    memset(recent_messages, 0, sizeof(recent_messages));
    last_send_too_long = false;
    last_snr = 0;
    time_sample_count = 0;
    memset(time_samples, 0, sizeof(time_samples));
  }

  float getFreqPref() const { return _prefs.freq; }
  uint8_t getTxPowerPref() const { return _prefs.tx_power_dbm; }
  float getBWPref() const { return _prefs.bw; }
  uint8_t getSFPref() const { return _prefs.sf; }
  uint8_t getCRPref() const { return _prefs.cr; }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;
    
    if (!_fs) {
      SerialPort.println("FATAL: Filesystem not initialized");
      return;
    }

    BaseChatMesh::begin();

  #if defined(NRF52_PLATFORM)
    IdentityStore store(fs, "");
  #elif defined(RP2040_PLATFORM)
    IdentityStore store(fs, "/identity");
    store.begin();
  #else
    IdentityStore store(fs, "/identity");
  #endif
    if (!store.load("_main", self_id, _prefs.node_name, sizeof(_prefs.node_name))) {  // legacy: node_name was from identity file
      // Need way to get some entropy to seed RNG
      SerialPort.println("No identity found. Generating new keypair...");
      SerialPort.println("Press ENTER to continue (timeout in 30s):");
      
      unsigned long start = millis();
      bool got_input = false;
      char c = 0;
      while (millis() - start < KEY_GEN_TIMEOUT_MS) {
        if (SerialPort.available()) {
          c = SerialPort.read();
          if (c == '\n' || c == '\r') {
            got_input = true;
            break;
          }
        }
        delay(10);  // Small delay to prevent busy-wait
      }
      
      if (!got_input) {
        SerialPort.println("Timeout - proceeding with key generation");
      }
      
      ((StdRNG *)getRNG())->begin(millis());

      self_id = mesh::LocalIdentity(getRNG());  // create new random identity
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
        self_id = mesh::LocalIdentity(getRNG()); count++;
      }
      
      if (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF) {
        SerialPort.println("WARNING: Generated identity may have reserved hash");
      }
      
      store.save("_main", self_id);
      SerialPort.println("Identity saved.");
    }

    // load persisted prefs
    if (_fs->exists("/node_prefs")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/node_prefs", "r");
    #else
      File file = _fs->open("/node_prefs");
    #endif
      if (file) {
        size_t bytes_read = file.read((uint8_t *) &_prefs, sizeof(_prefs));
        if (bytes_read != sizeof(_prefs)) {
          SerialPort.println("Warning: Node preferences file corrupted");
        }
        file.close();
      }
    }

    loadContacts();
    _public = addChannel("Public", PUBLIC_GROUP_PSK); // pre-configure Andy's public channel
    if (!_public) {
      SerialPort.println("Warning: Failed to add public channel");
    }
  }

  void savePrefs() {
    if (!checkFilesystem()) return;
    
    File file = openFileForWrite("/node_prefs");
    if (!file) {
      SerialPort.println("ERROR: Could not open preferences file for writing");
      return;
    }
    
      size_t written = file.write((const uint8_t *)&_prefs, sizeof(_prefs));
      if (written != sizeof(_prefs)) {
      SerialPort.println("ERROR: Failed to write preferences");
      }
      file.close();
  }

  void showWelcome() {
    updateTerminalTitle();  // Set initial terminal title
    
    SerialPort.println();
    SerialPort.printf("%s=== MeshCore Secure Chat ===%s\n", ansi(ANSI_BOLD), ansi(ANSI_RESET));
    SerialPort.println(FIRMWARE_VER_TEXT);
    SerialPort.println();
    SerialPort.printf("Name: %s\n", _prefs.node_name);
    SerialPort.print("Key: ");
    mesh::Utils::printHex(SerialPort, self_id.pub_key, PUB_KEY_SIZE);
    SerialPort.println();
    SerialPort.printf("Contacts: %d\n", getNumContacts());
    SerialPort.println();
    
    // First-run hints
    if (strcmp(_prefs.node_name, "NONAME") == 0) {
      SerialPort.printf("%sTip: Set your name with 'set name <yourname>'%s\n", 
                    ansi(ANSI_YELLOW), ansi(ANSI_RESET));
    }
    if (getNumContacts() == 0) {
      SerialPort.printf("%sTip: Import contacts with 'import <card>'%s\n", 
                    ansi(ANSI_YELLOW), ansi(ANSI_RESET));
    }
    
    SerialPort.printf("%sType 'help' or '?' for commands%s\n", ansi(ANSI_DIM), ansi(ANSI_RESET));
    SerialPort.println();
    showPrompt();
  }

  void sendSelfAdvert(int delay_millis) {
    auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
    if (pkt) {
      sendFlood(pkt, delay_millis);
    }
  }

  // ContactVisitor
  void onContactVisit(const ContactInfo& contact) override {
    SerialPort.printf("   %s - ", contact.name);
    char tmp[40];
    int32_t secs = contact.last_advert_timestamp - getRTCClock()->getCurrentTime();
    AdvertTimeHelper::formatRelativeTimeDiff(tmp, secs, false);
    SerialPort.println(tmp);
  }

  void handleCommand(const char* command) {
    while (*command == ' ') command++;  // skip leading spaces
    if (*command == 0) return;  // Empty command

    // Message commands
    if (memcmp(command, "send ", 5) == 0) {
      cmdSend(&command[5]);
    }
    // Contact list commands
    else if (memcmp(command, "list", 4) == 0) {
      int n = 0;
      if (command[4] == ' ') {
        n = _atoi(&command[5]);
        if (n < 0 || n > MAX_CONTACTS) {
          SerialPort.printf("%s   ERROR: Invalid count (max %d)%s\n", 
                        ansi(ANSI_RED), MAX_CONTACTS, ansi(ANSI_RESET));
        return;
        }
      }
      cmdList(n);
    } else if (strcmp(command, "contacts") == 0) {
      cmdList(0);
    } else if (strcmp(command, "history") == 0) {
      cmdHistory();
    }
    // Time commands
    else if (strcmp(command, "clock") == 0) {
      cmdClock();
    }
    // Recipient commands
    else if (memcmp(command, "to ", 3) == 0) {
      cmdSetRecipient(&command[3]);
    } else if (strcmp(command, "to") == 0) {
      cmdShowRecipient();
    }
    // Network commands
    else if (strcmp(command, "advert") == 0) {
      cmdAdvert();
    } else if (strcmp(command, "reset path") == 0) {
      cmdResetPath();
    }
    // Contact management
    else if (memcmp(command, "delete ", 7) == 0) {
      cmdDelete(&command[7]);
    } else if (memcmp(command, "rename ", 7) == 0) {
      const char* args = &command[7];
      const char* space = strchr(args, ' ');
      if (!space) {
        printError("Usage: rename <old_name> <new_name>");
        return;
      }
      
      char old_name[33];
      int old_len = space - args;
      if (old_len > 32) old_len = 32;
      memcpy(old_name, args, old_len);
      old_name[old_len] = 0;
      
      const char* new_name = space + 1;
      while (*new_name == ' ') new_name++;
      
      cmdRename(old_name, new_name);
    }
    // Card/import commands
    else if (memcmp(command, "card", 4) == 0) {
      cmdCard();
    } else if (memcmp(command, "import ", 7) == 0) {
      importCard(&command[7]);
    }
    // Configuration
    else if (memcmp(command, "set ", 4) == 0) {
      cmdSetConfig(&command[4]);
    }
    // Info commands
    else if (memcmp(command, "info", 4) == 0) {
      cmdInfo();
    } else if (memcmp(command, "status", 6) == 0) {
      cmdStatus();
    } else if (memcmp(command, "radio", 5) == 0) {
      cmdRadio();
    } else if (memcmp(command, "reboot", 6) == 0) {
      SerialPort.println("   Rebooting...");
      SerialPort.flush();
      delay(100);
      #if defined(ESP32)
        ESP.restart();
      #elif defined(NRF52_PLATFORM)
        NVIC_SystemReset();
      #elif defined(RP2040_PLATFORM)
        watchdog_reboot(0, 0, 0);
      #else
        // Fallback for other platforms
        void (*resetFunc)(void) = 0;
        resetFunc();
      #endif
    } else if (memcmp(command, "ver", 3) == 0) {
      SerialPort.println(FIRMWARE_VER_TEXT);
    } else if (memcmp(command, "help", 4) == 0 || strcmp(command, "?") == 0) {
      cmdHelp();
    } else {
      SerialPort.printf("%s   Unknown: '%s'%s (type 'help')\n", 
                    ansi(ANSI_YELLOW), command, ansi(ANSI_RESET));
    }
  }

  void loop() {
    BaseChatMesh::loop();

    int len = strlen(command);
    while (SerialPort.available() && len < sizeof(command)-2) {  // Leave room for null terminator
      char c = SerialPort.read();
      
      // Handle CR/LF line endings (support \r, \n, or \r\n)
      if (c == '\r' || c == '\n') {
        if (len > 0) {  // Only process non-empty commands
          SerialPort.println();  // Move to new line
          command[len] = 0;  // Ensure null termination
          
          last_send_too_long = false;  // Reset flag
          handleCommand(command);
          
          // Preserve buffer if the message was too long
          if (last_send_too_long) {
            len = strlen(command);  // Keep the buffer
            showPrompt();
            SerialPort.print(command);  // Re-display for editing
          } else {
            command[0] = 0;  // Clear buffer
          len = 0;
            showPrompt();
          }
        }
        
        // If this is CR, consume the following LF if present
        if (c == '\r' && SerialPort.available()) {
          char next = SerialPort.peek();
          if (next == '\n') {
            SerialPort.read();  // Consume the LF
          }
        }
        break;
      }
      
      // Handle backspace
      if (c == '\b' || c == 127) {
        if (len > 0) {
          len--;
          command[len] = 0;
          SerialPort.print("\b \b");  // Erase character on terminal
        }
        continue;
      }
      
      // Ignore control characters except for printable ASCII
      if (c >= 32 && c < 127) {
        command[len++] = c;
        command[len] = 0;  // Keep buffer null-terminated
        SerialPort.print(c);  // Echo character
      }
    }
    
    // Handle buffer full condition - safely truncate instead of erroring
    if (len >= sizeof(command)-2) {
      // Don't accept more input, but let user try to send or backspace
      // The validation in cmdSend will catch if it's too long for a message
    }
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

void setup() {
  initSerial();

  board.begin();

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_get_rng_seed());

#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  the_mesh.begin(InternalFS);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  the_mesh.begin(LittleFS);
#elif defined(ESP32)
  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS);
#else
  #error "need to define filesystem"
#endif

  radio_set_params(the_mesh.getFreqPref(), the_mesh.getBWPref(), the_mesh.getSFPref(), the_mesh.getCRPref());
  radio_set_tx_power(the_mesh.getTxPowerPref());

  the_mesh.showWelcome();
}

void loop() {
  the_mesh.loop();
  rtc_clock.tick();
}
