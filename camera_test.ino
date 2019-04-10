const byte camera = 5; //any digital pin
const byte picTime = 250; //time in ms to take a picture
const byte vidTime = 1500;//time in ms to take a video


void setup() {
  // put your setup code here, to run once:
  pinMode(camera, OUTPUT);
  takePicture();
  delay(1000);
  toggleVideo();
  delay(5000);
  toggleVideo();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void takePicture() {
  digitalWrite(camera, LOW);
  delay(picTime);
  digitalWrite(camera, HIGH);
}

void toggleVideo() {
  digitalWrite(camera, LOW);
  delay(vidTime);
  digitalWrite(camera, LOW);
}
