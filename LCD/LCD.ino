// include the library code:
#include <LiquidCrystal.h>

LiquidCrystal lcd(5, 7, 8, 9, 10, 11);

void printMode();
void printSpeed();
void printDirection();
void printInformation();

// initialize the library with the numbers of the interface pins
void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
}

void loop() 
{
  // prints the current robot data onto the LCD
  printInformation();  
  delay(1000);
}

void printInformation()
{
  lcd.clear();
  // prints the current mode onto the top row
  printMode();

  // prints the current speed onto the bottom left corner
  printSpeed();

  // prints the current direction onto the bottom right corner
  printDirection();
}

void printSpeed()
{
  int SPEED = 40;

  // set the cursor to column 0, line 1 
  lcd.setCursor(1,1);

  // print the numerical speed
  lcd.print(SPEED);

  // print the speed units
  lcd.print("cm/s");
}

void printDirection()
{
  int DIRECTION = 2;

  // set the cursor to column 
  lcd.setCursor(10,1);

  // print the direction
  switch(DIRECTION)
  {
    case(1): lcd.print("FWD"); break;
    case(2): lcd.print("BWD"); break;
  }
}

void printMode()
{
  int MODE = 2;
  // set the cursor to column 0, line 0
  lcd.setCursor(0,0);

  // print "MODE"
  lcd.print("MODE: ");
  
  switch(MODE)
  {
    case(1): lcd.print("AUTO"); break;
    case(2): lcd.print("PATH"); break;
    case(3): lcd.print("REMOTE"); break;
  }
}


