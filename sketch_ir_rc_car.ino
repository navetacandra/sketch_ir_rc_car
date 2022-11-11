// Start Import Library
#include <Servo.h>
#include <IRremote.h>
// End Import Library

// Start PIN Declare
const uint8_t ENA_R = 2;
const uint8_t IN1_R = 3;
const uint8_t IN2_R = 4;
const uint8_t IN3_R = 5;
const uint8_t IN4_R = 6;
const uint8_t ENB_R = 7;
const uint8_t ENA_L = 8;
const uint8_t IN1_L = 9;
const uint8_t IN2_L = 10;
const uint8_t IN3_L = 11;
const uint8_t IN4_L = 12;
const uint8_t ENB_L = 13;
// ---------------------- //
const uint8_t SERVO = A0;
const uint8_t IR_REMOTE = A1;
const uint8_t IR_L = A2;
const uint8_t IR_R = A3;
// End PIN Declare

// Start Library Uses
IRrecv remote(IR_REMOTE);
Servo myservo;
decode_results signal;
// End Library Uses

// Start IR Code Declare
const uint32_t substract_code = 16738455;
const uint32_t sharp_code = 16756815;
const uint32_t zero_code = 16750695;
const uint32_t one_code = 16753245;
const uint32_t two_code = 16736925;
const uint32_t three_code = 16769565;
const uint32_t four_code = 16720605;
const uint32_t five_code = 16712445;
const uint32_t six_code = 16761405;
const uint32_t seven_code = 16769055;
const uint32_t eight_code = 16754775;
const uint32_t nine_code = 16748655;
const uint32_t forward_code = 16718055;
const uint32_t back_code = 16730805;
const uint32_t right_code = 16734885;
const uint32_t left_code = 16716015;
const uint32_t grip_code = 16726215;
const uint32_t repeated_code = 4294967295;
// End IR Code Declare

// Start Movement Status
int speed = 0;
bool follow_line = false;
bool one = false;
bool two = false;
bool forward = false;
bool back = false;
bool right = false;
bool left = false;
bool grip = false;
// End Movement Status

void setup() {
  Serial.begin(9600);     // Start Serial Monitor
  myservo.attach(SERVO);  // Attach To Servo PIN
  remote.enableIRIn();    // Start IR Receiver To Get Signal
  myservo.write(0);       // Write Servo To ZERO (RESET)

  // Start Declare Output PIN
  pinMode(ENA_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(IN3_R, OUTPUT);
  pinMode(IN4_R, OUTPUT);
  pinMode(ENB_R, OUTPUT);
  pinMode(ENA_L, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN3_L, OUTPUT);
  pinMode(IN4_L, OUTPUT);
  pinMode(ENB_L, OUTPUT);
  // End Declare Output PIN

  // Start Declare Input PIN
  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);
  // End Declare Input PIN
}

void loop() {
  smart_car();  // Run smart_car Function

  // Check Grip Status
  if (grip) myservo.write(60);  // Write Servo To 60 When Grip Is TRUE
  else myservo.write(0);        // Write Servo To 0 (ZERO) When Grip Is FALSE

  // Check Follow Line Mode
  if (follow_line) smart_car_follow_line();  // Run Line Detection

  // Check Movement Status
  if (forward) move_forward();
  else if (back) move_back();
  else if (right) {
    if (one) rotate_right();
    if (two) move_right();
  } else if (left) {
    if (one) rotate_left();
    if (two) move_left();
  } else stop();
}

void smart_car() {
  // Automatically Enter Mode one
  if (!one && !two) one = true;
  // Set Movement Speed
  if (follow_line) speed = 0;  // Set Speed To Max When Follow Line Mode
  else speed = 8;              // Set Speed To 8 When Follow Line Mode

  // Check IR Remote Signal
  if (remote.decode(&signal)) {
    // Print Decoded Signal Value
    Serial.println(signal.value);
    // Check Is Repeated Button
    if (signal.value != repeated_code) {
      // Set Mode one When Button 1 Pressed
      if (signal.value == one_code) {
        one = true;
        two = false;
      }
      // Set Mode two When Button 2 Pressed
      if (signal.value == two_code) {
        one = false;
        two = true;
      }
      // Toggle Follow Line Mode When Button 3 Pressed
      if (signal.value == three_code) {
        // Check Follow Line Status
        if (follow_line == false) {
          // Toggle Follow Line Mode TRUE When Previous Status FALSE
          one = true;
          two = false;
          follow_line = true;
          forward = false;
          back = false;
          right = false;
          left = false;
        } else {
          // Toggle Follow Line Mode FALSE When Previous Status TRUE
          one = true;
          two = false;
          follow_line = false;
          forward = false;
          back = false;
          right = false;
          left = false;
        }
      }
      // Rotate 45 Degree To Right When Button 5 Pressed
      if (signal.value == five_code && !follow_line) run_rotate_45_r();
      // Rotate 45 Degree To Left When Button 8 Pressed
      if (signal.value == eight_code && !follow_line) run_rotate_45_l();
      // Rotate 90 Degree To Right When Button 4 Pressed
      if (signal.value == four_code && !follow_line) run_rotate_90_r();
      // Rotate 90 Degree To Left When Button 7 Pressed
      if (signal.value == seven_code && !follow_line) run_rotate_90_l();
      // Rotate 180 Degree To Rightt When Button * Pressed
      if (signal.value == substract_code && !follow_line) run_rotate_180_l();
      // Rotate 180 Degree To Left When Button # Pressed
      if (signal.value == sharp_code && !follow_line) run_rotate_180_r();
      // Rotate 360 Degree Clockwise When Button 0 Pressed
      if (signal.value == zero_code && !follow_line) run_rotate_360();
      // Run Forward When Button ↑ Pressed
      if (signal.value == forward_code && !follow_line) forward = true;
      // Run Backward When Button ↓ Pressed
      if (signal.value == back_code && !follow_line) back = true;
      // Run Right When Button → Pressed
      if (signal.value == right_code && !follow_line) right = true;
      // Run Left When Button ← Pressed
      if (signal.value == left_code && !follow_line) left = true;
      // Toggle Grip When Button OK Pressed
      if (signal.value == grip_code && !follow_line) {
        // Check Grip Status
        if (grip == false) {
          // Toggle Grip To TRUE When Previous Status FALSE
          grip = true;
        } else {
          // Toggle Grip To FALSE When Previous Status TRUE
          grip = false;
        }
      }
    }
    // Resume IR To Recieve Signal
    remote.resume();
    // Calc Delay
    delay(100 - (speed * 5));
  } else {
    // Set All Movement Status To FALSE When No Signal Recieved
    forward = false;
    back = false;
    right = false;
    left = false;
  }
}

void smart_car_follow_line() {
  // Get IR Value
  uint8_t IR_R_VAL = digitalRead(IR_R);
  uint8_t IR_L_VAL = digitalRead(IR_L);

  // Run Forward When No Line Deteceted
  if (IR_R_VAL == 0 && IR_L_VAL == 0) {
    move_forward();
    delayMicroseconds(1000);
    stop();
  }
  // Rotate Left When Left Line Deteceted
  if (IR_R_VAL == 0 && IR_L_VAL == 1) {
    rotate_left();
    delayMicroseconds(1000);
    stop();
  }
  // Rotate Right When Right Line Deteceted
  if (IR_R_VAL == 1 && IR_L_VAL == 0) {
    rotate_right();
    delayMicroseconds(1000);
    stop();
  }
  // Stop When Left And Right Line Detected
  if (IR_R_VAL == 1 && IR_L_VAL == 1) stop();
}

void run_rotate_45_r() {
  // Declare Needed Variabel
  bool is_rotate_45_r = false;
  int rotate_45_r = 0;

  is_rotate_45_r = true;
  rotate_45_r = millis();
  // Rotate To Right In 160ms
  while (is_rotate_45_r) {
    if (millis() - rotate_45_r < 160) {
      rotate_right();
      delay(10 / 3);
      stop();
    } else {
      is_rotate_45_r = false;
      rotate_45_r = 0;
    }
  }
}

void run_rotate_45_l() {
  // Declare Needed Variabel
  bool is_rotate_45_l = false;
  int rotate_45_l = 0;

  is_rotate_45_l = true;
  rotate_45_l = millis();
  // Rotate To Left In 160ms
  while (is_rotate_45_l) {
    if (millis() - rotate_45_l < 160) {
      rotate_left();
      delay(10 / 3);
      stop();
    } else {
      is_rotate_45_l = false;
      rotate_45_l = 0;
    }
  }
}

void run_rotate_90_r() {
  // Declare Needed Variabel
  bool is_rotate_90_r = false;
  int rotate_90_r = 0;

  is_rotate_90_r = true;
  rotate_90_r = millis();
  // Rotate To Right In 320ms
  while (is_rotate_90_r) {
    if (millis() - rotate_90_r < 320) {
      rotate_right();
      delay(10 / 3);
      stop();
    } else {
      is_rotate_90_r = false;
      rotate_90_r = 0;
    }
  }
}

void run_rotate_90_l() {
  // Declare Needed Variabel
  bool is_rotate_90_l = false;
  int rotate_90_l = 0;

  is_rotate_90_l = true;
  rotate_90_l = millis();
  // Rotate To Right In 320ms
  while (is_rotate_90_l) {
    if (millis() - rotate_90_l < 320) {
      rotate_left();
      delay(10 / 3);
      stop();
    } else {
      is_rotate_90_l = false;
      rotate_90_l = 0;
    }
  }
}

void run_rotate_180_r() {
  // Declare Needed Variabel
  bool is_rotate_180_r = false;
  int rotate_180_r = 0;

  is_rotate_180_r = true;
  rotate_180_r = millis();
  // Rotate To Right In 740ms
  while (is_rotate_180_r) {
    if (millis() - rotate_180_r < 740) {
      rotate_right();
      delay(10 / 3);
      stop();
    } else {
      is_rotate_180_r = false;
      rotate_180_r = 0;
    }
  }
}

void run_rotate_180_l() {
  // Declare Needed Variabel
  bool is_rotate_180_l = false;
  int rotate_180_l = 0;

  is_rotate_180_l = true;
  rotate_180_l = millis();
  // Rotate To Left In 740ms
  while (is_rotate_180_l) {
    if (millis() - rotate_180_l < 740) {
      rotate_left();
      delay(10 / 3);
      stop();
    } else {
      is_rotate_180_l = false;
      rotate_180_l = 0;
    }
  }
}

void run_rotate_360() {
  // Declare Needed Variabel
  bool is_rotate_360 = false;
  int rotate_360 = 0;

  is_rotate_360 = true;
  rotate_360 = millis();
  // Rotate To Right In 1550ms
  while (is_rotate_360) {
    if (millis() - rotate_360 < 1550) {
      rotate_right();
      delay(10 / 3);
      stop();
    } else {
      is_rotate_360 = false;
      rotate_360 = 0;
    }
  }
}

void stop() {
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENB_R, LOW);
  digitalWrite(ENA_L, LOW);
  digitalWrite(ENB_L, LOW);
}

void move_forward() {
  delay(5);
  run_right_wheels(1, 1);
  run_left_wheels(1, 1);

  if (speed != 0) {
    int move_count = 0;
    while (move_count < 5) {
      activate_wheels();
      delay(speed);
      stop();
      move_count = move_count + 1;
    }
  } else {
    activate_wheels();
  }
}

void move_back() {
  delay(5);
  run_right_wheels(0, 0);
  run_left_wheels(0, 0);

  if (speed != 0) {
    int move_count = 0;
    while (move_count < 5) {
      activate_wheels();
      delay(speed);
      stop();
      move_count = move_count + 1;
    }
  } else {
    activate_wheels();
  }
}

void move_left() {
  delay(5);
  run_right_wheels(1, 0);
  run_left_wheels(0, 1);

  if (speed != 0) {
    int move_count = 0;
    while (move_count < 5) {
      activate_wheels();
      delay(speed);
      stop();
      move_count = move_count + 1;
    }
  } else {
    activate_wheels();
  }
}

void move_right() {
  delay(5);
  run_right_wheels(0, 1);
  run_left_wheels(1, 0);

  if (speed != 0) {
    int move_count = 0;
    while (move_count < 5) {
      activate_wheels();
      delay(speed);
      stop();
      move_count = move_count + 1;
    }
  } else {
    activate_wheels();
  }
}

void rotate_right() {
  delay(5);
  run_right_wheels(0, 0);
  run_left_wheels(1, 1);

  if (speed != 0) {
    int move_count = 0;
    while (move_count < 5) {
      activate_wheels();
      delay(speed);
      stop();
      move_count = move_count + 1;
    }
  } else {
    activate_wheels();
  }
}

void rotate_left() {
  delay(5);
  run_right_wheels(1, 1);
  run_left_wheels(0, 0);

  if (speed != 0) {
    int move_count = 0;
    while (move_count < 5) {
      activate_wheels();
      delay(speed);
      stop();
      move_count = move_count + 1;
    }
  } else {
    activate_wheels();
  }
}

void activate_wheels() {
  digitalWrite(ENA_R, HIGH);
  digitalWrite(ENB_R, HIGH);
  digitalWrite(ENA_L, HIGH);
  digitalWrite(ENB_L, HIGH);
}

void run_right_wheels(int front_wheel, int back_wheel) {
  if (front_wheel == 1) {
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  } else {
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  }

  if (back_wheel == 1) {
    digitalWrite(IN3_R, LOW);
    digitalWrite(IN4_R, HIGH);
  } else {
    digitalWrite(IN3_R, HIGH);
    digitalWrite(IN4_R, LOW);
  }
}

void run_left_wheels(int front_wheel, int back_wheel) {
  if (front_wheel == 1) {
    digitalWrite(IN3_L, HIGH);
    digitalWrite(IN4_L, LOW);
  } else {
    digitalWrite(IN3_L, LOW);
    digitalWrite(IN4_L, HIGH);
  }

  if (back_wheel == 1) {
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  } else {
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }
}