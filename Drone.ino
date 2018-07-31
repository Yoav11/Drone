float currentTime, previousTime, elapsedTime;

void setup() {
  setupMPU();
  PIDSetup();
  setupRemote();
}
void loop() {
  loopMPU();
  PIDControl();
  loopRemote();
}
