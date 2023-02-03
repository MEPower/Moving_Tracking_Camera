#define ARDUINO 101

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    delay(500);
    if (Serial.available() > 1)
    {
        char c = Serial.read();
        char d = Serial.read();
        Serial.print(c);
        Serial.print(d);
    }
    Serial.print('h');

    int i = 96;
    Serial.print(i);
}
