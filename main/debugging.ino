// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//

// DEBUGGER FUNCTIONS
// These include functions useful for debugging

bool shouldPrint(unsigned long current_time, float freq) {
  static unsigned long print_counter = 0;

  long unsigned time_micros = 1.0f / freq * SEC_TO_MICROSEC;

  if (current_time - print_counter > time_micros) {
    print_counter = micros();
    return true;
  } else {
    return false;
  }
}

void printDebug(const char name[], float value) {
  Serial.print(name);
  Serial.print(F(":"));
  Serial.print(value);
}

void printNewLine() {
  Serial.println("");
}
