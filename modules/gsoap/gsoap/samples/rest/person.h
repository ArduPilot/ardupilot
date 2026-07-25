#import "custom/struct_tm_date.h"
struct Person {
  const char *name;
  xsd__date dob;
  enum Gender { MALE, FEMALE } gender;
};
