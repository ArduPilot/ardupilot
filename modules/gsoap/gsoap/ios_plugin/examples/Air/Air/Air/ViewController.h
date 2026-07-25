//
//  ViewController.h
//  

#import <UIKit/UIKit.h>

@interface ViewController : UIViewController{
    UITextField* country_name;
    UITextView* showResults;
}
@property (strong, nonatomic) IBOutlet UITextField *country_name;
@property (strong, nonatomic) IBOutlet UITextView *showResults;


-(IBAction) buttonPressed;

@end

