//
// File: ViewController.h
//

#import <UIKit/UIKit.h>

@interface ViewController : UIViewController {
    UITextField *op1; // operand1
    UITextField *op2; // operand2
    
}
@property (nonatomic, retain) IBOutlet UITextField *op1;
@property (nonatomic, retain) IBOutlet UITextField *op2;

- (IBAction) buttonPressed;
@end

