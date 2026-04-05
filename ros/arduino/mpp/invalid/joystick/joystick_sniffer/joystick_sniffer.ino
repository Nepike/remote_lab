int val = 0;

void setup() 
{
  Serial.begin(9600);
  
  pinMode(A2, INPUT); // 2
  pinMode(A3, INPUT); // 4
  pinMode(A4, INPUT); // 5
  pinMode(A5, INPUT); // 6
  pinMode(A6, INPUT); // 7
  pinMode(A7, INPUT); // 8
}

float D2V(int d)
{
  return (d/1024.) * 5.0;
}

void loop() 
{
  val = analogRead(3);
  Serial.print(D2V(val));
  Serial.print(" ");
  val = analogRead(5);
  Serial.print(D2V(val));
  Serial.print(" ");
  val = analogRead(6);
  Serial.print(D2V(val));
  Serial.print(" ");
  val = analogRead(9);
  Serial.print(D2V(val));
  Serial.print(" ");
  val = analogRead(10);
  Serial.print(D2V(val));
  Serial.print(" ");
  val = analogRead(11);
  Serial.println(D2V(val));
  
  delay(100);
}
