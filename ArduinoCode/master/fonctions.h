void ar() //arriére
{
  digitalWrite(DA, HIGH);
  digitalWrite(DB, LOW);
  digitalWrite(GA, HIGH);
  digitalWrite(GB, LOW);
}

void av() //avant
{
  digitalWrite(DA, LOW);
  digitalWrite(DB, HIGH);
  digitalWrite(GA, LOW);
  digitalWrite(GB, HIGH);
}

void g()//gauche
{
  digitalWrite(DA, LOW);
  digitalWrite(DB, HIGH);
  digitalWrite(GA, HIGH);
  digitalWrite(GB, LOW);
}

void d()//droite
{
  digitalWrite(DA, HIGH);
  digitalWrite(DB, LOW);
  digitalWrite(GA, LOW);
  digitalWrite(GB, HIGH);
}
