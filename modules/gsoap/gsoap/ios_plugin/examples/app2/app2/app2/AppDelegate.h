//
//AppDelegate.h
//

#import <UIKit/UIKit.h>

@class ViewController;
@interface AppDelegate : UIResponder <UIApplicationDelegate>{
    UIWindow *window;
    ViewController *viewController;
}

@property (strong, nonatomic) UIWindow *window;
@property (nonatomic, retain) IBOutlet ViewController *viewController;

@end


