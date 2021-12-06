// BSFrance LoRa32u4 v1.1 / Adafruit feather 32u4
#define SCK     15   // GPIO5  -- SX1278's SCK
#define MISO    14   // GPIO19 -- SX1278's MISO
#define MOSI    16   // GPIO27 -- SX1278's MOSI
#define SS      8    // GPIO18 -- SX1278's CS
#define RST     4    // GPIO14 -- SX1278's RESET
#define DI0     7    // GPIO26 -- SX1278's IRQ(Interrupt Request)

// MISC defines
#define BAND      915E6     // radio frequency band USA= 915E6
#define BAUD_RATE 9600      // Serial connection baud rate, for debugging etc
bool LINEAR_ON = true;
#define MOVE_SPEED 5

// SCREEN STUFF
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// build screen instance, default: "display"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// joysticks
#define L_X_PIN A3 // x
#define L_Y_PIN A2 // y
#define L_B_PIN 5  // button

#define R_X_PIN A1 // x
#define R_Y_PIN A0 // y
#define R_B_PIN 6  // button

// slider + menu buttons
#define POT_PIN  A4
#define A_PIN  11
#define B_PIN 9
#define C_PIN   12
#define D_PIN 10

// button logic bools
bool   L_tog = false; // left stick button
bool   R_tog = false; // right stick button
bool   A_tog = false; // A button
bool   B_tog = false; // B button
bool   C_tog = false; // C button
bool   D_tog = false; // D button

Bounce L_button;
Bounce R_button;
Bounce A_button; // top-left
Bounce B_button; // bottom-left
Bounce C_button; // top-right
Bounce D_button; // bottom-right

// array to hold control data values
#define PACKET_SIZE 11
enum CONTROL_SIGNAL {
       L_xpos,    // left stick x
       L_ypos,    // left stick y
       L_click,   // left stick button
       R_xpos,    // right stick x
       R_ypos,    // right stick y
       R_click,   // right stick button
       P_pos,     // slider (pot) postion
       A_click,   // select button
       B_click,   // back button
       C_click,   // up button
       D_click    // down button
     };
byte data_out[PACKET_SIZE];

// PID STUFF
float Ki, Kp, Kd = 0.0;

typedef struct PID_data_t {
  float p_val;
  float i_val;
  float d_val;
};

typedef union PID_packet_t {
  PID_data_t data;
  uint8_t bytes_out[sizeof(PID_data_t)];
};
PID_packet_t pid_out;
#define PID_PACKET_SIZE sizeof(pid_out)


// Timer stuff
#define DATA_CYCLE_TIME 60000 // in micros
#define DRAW_CYCLE_TIME 100000 // in micros
unsigned long prev_data_time = 0;
unsigned long prev_draw_time = 0;
unsigned long curr_data_time;
unsigned long curr_draw_time;

static const unsigned char PROGMEM WOLF_BMP[] =
{ B00110000, B00000000, B00000000, B01100000,
  B00101100, B00000000, B00000001, B10100000,
  B00100011, B00000000, B00000110, B00100000,
  B00100000, B11000000, B00001000, B00100000,
  B00100000, B00101111, B11101000, B00100000,
  B00100000, B00010000, B00010000, B00100000,
  B00010000, B00000000, B00000000, B00111000,
  B00011000, B00000000, B00000000, B00000100,
  B00001000, B00100010, B00000000, B00000010,
  B00001000, B00010100, B00010001, B00111111,
  B00010000, B00001000, B00001010, B00010000,
  B00100000, B00010100, B00000100, B00001000,
  B01100000, B00100010, B00001010, B00000100,
  B00011000, B00000000, B00010001, B00111110,
  B00000100, B00000000, B00000000, B00100000,
  B00001000, B00000000, B00000000, B00100000,
  B00010000, B00000000, B00000000, B00100000,
  B00100000, B00000000, B00000000, B00100000,
  B01111100, B00000000, B00000000, B00010000,
  B00000011, B10000000, B00000000, B00010000,
  B00000100, B01100000, B00000000, B00010000,
  B00000100, B00010000, B00000000, B00001000,
  B00001000, B00101000, B00000000, B00001000,
  B00001000, B01000110, B00000000, B00001100,
  B00001000, B01000001, B00000000, B00000100,
  B00001000, B01000000, B10000000, B00000010,
  B00001000, B01000000, B01100000, B00000111,
  B00000100, B00100000, B00010000, B00001111,
  B00000100, B00100000, B00001100, B00011111,
  B00000010, B00100000, B00000011, B10111110,
  B00000001, B11000000, B00000000, B01111100,
  B00000000, B00000000, B00000000, B00011000 };

void full_init_routine() {
  // Serial connection init
  Serial.begin(BAUD_RATE);
  Serial.println();
  Serial.println("Serial connection established...");
  
  // LoRa init
  Serial.println("Attempting to start LoRa...");
  LoRa.setPins(SS,RST,DI0); // set correct pins for logic level converters etc
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);              // loop forever if LoRa init fails
  }
  Serial.println("LoRa started successfully!");

  // OLED init
  Serial.println("Attempting to start OLED...");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // wolf logo splash screen
  display.clearDisplay(); // clear out adafruit splash
  display.drawBitmap(47, 15, WOLF_BMP, 32, 32, 1);
  display.display();
  delay(2500);
  display.clearDisplay();

  // screen text stuff
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setTextWrap(false);

  // initialize button pins
  L_button.attach(L_B_PIN, INPUT_PULLUP);
  R_button.attach(R_B_PIN, INPUT_PULLUP);
  A_button.attach(A_PIN, INPUT_PULLUP);
  B_button.attach(B_PIN, INPUT_PULLUP);
  C_button.attach(C_PIN, INPUT_PULLUP);
  D_button.attach(D_PIN, INPUT_PULLUP);
  L_button.interval(1);
  R_button.interval(1);
  A_button.interval(1);
  B_button.interval(1);
  C_button.interval(1);
  D_button.interval(1);

  // set all data to initial defaults
  data_out[L_xpos]  = 127;  // L stick
  data_out[L_ypos]  = 127;
  data_out[L_click] = 0;
  data_out[R_xpos]  = 127;  // R stick
  data_out[R_ypos]  = 127;
  data_out[R_click] = 0;
  data_out[P_pos]   = 0;    // throt
  data_out[A_click] = 0;
  data_out[B_click] = 0;
  data_out[C_click] = 0;
  data_out[D_click] = 0;

  // LOAD SAVED PID VALS FROM EEPROM HERE
  EEPROM_readAnything(0, pid_out.data);
  Kp = pid_out.data.p_val;
  Ki = pid_out.data.i_val;
  Kd = pid_out.data.d_val;
}

// ------ USEFUL / DEBUG ------ //

void disarmPots() {
  data_out[L_xpos]  = 127;  // L stick
  data_out[L_ypos]  = 127;
  data_out[R_xpos]  = 127;  // R stick
  data_out[R_ypos]  = 127;
  data_out[P_pos]   = 0;    // throt
}

void printSerialData() {
  Serial.print("LX:");
  Serial.print(data_out[L_xpos]);
  Serial.print(" ");
  Serial.print("LY:");
  Serial.print(data_out[L_ypos]);
  Serial.print(" ");
  Serial.print("LB:");
  Serial.print(data_out[L_click]);
  Serial.print(" ");
  Serial.print("RX:");
  Serial.print(data_out[R_xpos]);
  Serial.print(" ");
  Serial.print("RY:");
  Serial.print(data_out[R_ypos]);
  Serial.print(" ");
  Serial.print("RB:");
  Serial.print(data_out[R_click]);
  Serial.print(" ");
  Serial.print("P:");
  Serial.print(data_out[P_pos]);
  Serial.print(" ");
  Serial.print("S:");
  Serial.print(data_out[A_click]);
  Serial.print(" ");
  Serial.print("B:");
  Serial.print(data_out[B_click]);
  Serial.print(" ");
  Serial.print("U:");
  Serial.print(data_out[C_click]);
  Serial.print(" ");
  Serial.print("D:");
  Serial.println(data_out[D_click]);
}

void printSerialPID() {
  Serial.print("Kp:");
  Serial.print(Kp);
  Serial.print(" Ki:");
  Serial.print(Ki);
  Serial.print(" Kd:");
  Serial.println(Kd);
  Serial.print("p_out:");
  Serial.print(pid_out.data.p_val);
  Serial.print(" i_out:");
  Serial.print(pid_out.data.i_val);
  Serial.print(" d_out:");
  Serial.println(pid_out.data.d_val);
}
