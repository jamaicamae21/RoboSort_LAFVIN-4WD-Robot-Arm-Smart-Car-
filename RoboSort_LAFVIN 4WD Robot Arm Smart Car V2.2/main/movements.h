void Rotate_Left(int speed)
{
    digitalWrite(2, LOW);
    analogWrite(5, speed + 30);
    digitalWrite(4, LOW);
    analogWrite(6, speed + 30);
}

void Rotate_Right(int speed)
{
    digitalWrite(2, HIGH);
    analogWrite(5, speed + 30);
    digitalWrite(4, HIGH);
    analogWrite(6, speed + 30);
}

void Move_Forward(int speed)
{
    digitalWrite(2, HIGH);
    analogWrite(5, speed);
    digitalWrite(4, LOW);
    analogWrite(6, speed);
}

void Move_Backward(int speed)
{
    digitalWrite(2, LOW);
    analogWrite(5, speed);
    digitalWrite(4, HIGH);
    analogWrite(6, speed);
}

void Stop()
{
    digitalWrite(2, LOW);
    analogWrite(5, 0);
    digitalWrite(4, HIGH);
    analogWrite(6, 0);
}