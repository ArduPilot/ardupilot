#include "FL/fl_ask.H"

#include "UserInterface.hpp"


int main(int argc, char **argv)
{
   Fl::scheme("plastic");
   UserInterface ui;
   ui.show();
   return Fl::run();
}
