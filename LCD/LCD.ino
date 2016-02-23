// include the library code:
#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// initialize the library with the numbers of the interface pins
void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
}

void loop() 
{
  // prints the current robot data onto the LCD
  printInformation();

  
}

void printInformation()
{
  // prints the current mode onto the top row
  printMode();

  // prints the current speed onto the bottom left corner
  printSpeed();

  // prints the current direction onto the bottom right corner
  printDirection();
}

}
void printSpeed()
{
  int SPEED = 40;

  // set the cursor to column 0, line 1 
  lcd.setCursor(1,1);

  // print the numerical speed
  lcd.print("%dcm/s",SPEED);
}

void printDirection()
{
  int DIRECTION = 1;

  // set the cursor to column 
  lcd.setCursor(10,1);

  // print the direction
  switch(DIRECTION)
  {
    case(1): lcd.print("FWD");
    case(2): lcd.print("BWD");
  }
}

void printMode()
{
  int MODE = 1;
  // set the cursor to column 0, line 0
  lcd.setCursor(0,0);

  // print "MODE"
  lcd.print("MODE: ");

  lcd.setCursor(0,6);
  switch(MODE)
  {
    case(1): lcd.print("AUTO");
    case(2): lcd.print("PATH");
    case(3): lcd.print("REMOTE");
  }
}


