void setup() 
{
  // put your setup code here, to run once:
  pinMode(3, OUTPUT); // 2
  pinMode(5, OUTPUT); // 4
  pinMode(6, OUTPUT); // 5
  pinMode(9, OUTPUT); // 6
  pinMode(10, OUTPUT); // 7
  pinMode(11, OUTPUT); // 8
}

int V2Digital(float v)
{
  return (v/5)*255;
}

void loop() 
{
  analogWrite(3,V2Digital(2.5));
  analogWrite(5,V2Digital(2.5));
  analogWrite(6,V2Digital(2.5));
  analogWrite(9,V2Digital(2.5));
  analogWrite(10,V2Digital(2.5));
  analogWrite(11,V2Digital(0.0));
  
  delay(10);
}
