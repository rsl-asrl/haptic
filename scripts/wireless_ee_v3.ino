#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <DynamixelShield.h>

// ---------- WiFi ----------
char ssid[] = "Garage";
char pass[] = "Robotz0nly";

WiFiUDP Udp;

// Ports
const unsigned int EE_LOCAL_PORT   = 4211; // EE listens here for master position
const unsigned int FORCE_SEND_PORT = 4210; // wearable listens here for force
IPAddress wearableIP(192, 168, 0, 107);    // FIX: updated wearable IP

// ---------- DYNAMIXEL ----------
DynamixelShield dxl;
const float PROTOCOL_VERSION = 2.0;
const uint8_t ID_FOLLOW = 2;

// ---------- FSR ----------
const int FSR_1 = A1;
const int FSR_2 = A2;

// Moving average parameters
const int FSR_WINDOW = 5;
int  fsrHistory[FSR_WINDOW];
int  fsrIndex = 0;
bool fsrFilled = false;
long fsrSum   = 0;

// Outlier rejection
const int FSR_OUTLIER_LIMIT = 1023;
int lastGoodFSR = 0;

// Hysteresis thresholds (on/off for forceActive)
const int FSR_ON_THRESHOLD  = 20;
const int FSR_OFF_THRESHOLD = 10;

// Timing
const unsigned long FSR_SEND_INTERVAL_MS  = 20;  // 50 Hz for F packets when active
const unsigned long POS_APPLY_INTERVAL_MS = 5;   // ~200 Hz servo updates

unsigned long lastFsrSend   = 0;
unsigned long lastPosUpdate = 0;

// Buffers
char recvBuf[32];
float followerGoal = 130.0;  // safe default: closed position
int   FSR_AVG = 0;
bool  forceActive = false;

// --- FSR filtering helpers ---

int readRawFSR() {
  int raw1 = analogRead(FSR_1);
  int raw2 = analogRead(FSR_2);
  return (raw1 + raw2) / 2;
}

int getSmoothedFSR() {
  int raw = readRawFSR();

  if (!fsrFilled && fsrIndex == 0 && lastGoodFSR == 0) {
    for (int i = 0; i < FSR_WINDOW; i++) fsrHistory[i] = raw;
    fsrFilled   = true;
    fsrIndex    = 0;
    fsrSum      = (long)raw * FSR_WINDOW;
    lastGoodFSR = raw;
    return raw;
  }

  if (abs(raw - lastGoodFSR) > FSR_OUTLIER_LIMIT) raw = lastGoodFSR;

  fsrSum -= fsrHistory[fsrIndex];
  fsrHistory[fsrIndex] = raw;
  fsrSum += raw;
  fsrIndex = (fsrIndex + 1) % FSR_WINDOW;
  fsrFilled = true;

  int avg = (int)(fsrSum / FSR_WINDOW);
  lastGoodFSR = avg;
  return avg;
}

void updateForceActive() {
  if (!forceActive && FSR_AVG > FSR_ON_THRESHOLD) {
    forceActive = true;
  } else if (forceActive && FSR_AVG < FSR_OFF_THRESHOLD) {
    forceActive = false;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.println("EE: Connecting to WiFi...");
    status = WiFi.begin(ssid, pass);
    delay(200);
  }

  Serial.print("EE connected. IP: ");
  Serial.println(WiFi.localIP());

  Udp.begin(EE_LOCAL_PORT);
  Serial.print("EE listening on UDP port ");
  Serial.println(EE_LOCAL_PORT);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL_VERSION);
  dxl.torqueOn(ID_FOLLOW);

  Serial.println("End Effector Ready (FSR filtered).");
}

void loop() {
  unsigned long now = millis();

  // --- 1. Filter FSR ---
  FSR_AVG = getSmoothedFSR();
  updateForceActive();

  static unsigned long lastPrint = 0;
  if (now - lastPrint > 50) {
    lastPrint = now;
    Serial.print("FSR_AVG_smooth = ");
    Serial.print(FSR_AVG);
    Serial.print(" | forceActive = ");
    Serial.print(forceActive ? "YES" : "NO");
    Serial.print(" | followerGoal = ");
    Serial.println(followerGoal, 1);
  }

  // --- 2. Send F: packets only when forceActive and rate-limited ---
  if (forceActive && (now - lastFsrSend >= FSR_SEND_INTERVAL_MS)) {
    lastFsrSend = now;
    char buf[16];
    sprintf(buf, "F:%d", FSR_AVG);
    Udp.beginPacket(wearableIP, FORCE_SEND_PORT);
    Udp.write(buf);
    Udp.endPacket();
  }

  // --- 3. Receive master position ---
  // FIX: only update followerGoal when force is NOT active so the EE
  //      holds its contact position during haptic feedback instead of
  //      chasing the wearable and undermining the virtual spring.
  int packetSize;
  while ((packetSize = Udp.parsePacket()) > 0) {
    int len = Udp.read(recvBuf, sizeof(recvBuf) - 1);
    if (len > 0) {
      recvBuf[len] = '\0';
      float newGoal = atof(recvBuf);
      newGoal = constrain(newGoal, 130.0, 250.0);
      if (!isnan(newGoal) && !forceActive) {   // freeze when force is active
        followerGoal = newGoal;
      }
    }
  }

  // --- 4. Apply latest follower goal ---
  if (now - lastPosUpdate >= POS_APPLY_INTERVAL_MS) {
    lastPosUpdate = now;
    float safeGoal = constrain(followerGoal, 130.0, 250.0);
    dxl.setGoalPosition(ID_FOLLOW, safeGoal, UNIT_DEGREE);
  }

  // no delay()
}
