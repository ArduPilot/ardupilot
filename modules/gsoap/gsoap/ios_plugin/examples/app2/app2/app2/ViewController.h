//
//  ViewController.h
// 
#import <UIKit/UIKit.h>

@interface ViewController : UIViewController{
    UITextField* IPAddress;
}

@property (strong, nonatomic) IBOutlet UITextField *IPAddress;

- (IBAction) buttonPressed;
@end

