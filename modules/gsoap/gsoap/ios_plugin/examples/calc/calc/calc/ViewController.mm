//
//ViewController.mm
//

#import "ViewController.h"
#import "soapStub.h"
#import "gsoapios.h"

@implementation ViewController


@synthesize op1; // Set or get opreand1
@synthesize op2; // Set or get opreand1

- (IBAction)buttonPressed:(id)sender {
}

- (IBAction) buttonPressed
{
    double x = [op1.text doubleValue];
    double y = [op2.text doubleValue];
    
    struct soap *soap = soap_new1(SOAP_IO_DEFAULT|SOAP_IO_KEEPALIVE|SOAP_XML_INDENT|SOAP_XML_STRICT);
    
    if (soap_register_plugin(soap, soap_ios) == SOAP_OK) {
        // Specify the timeout interval (optional) to 45 seconds instead of
        // the default 60 seconds
        soap_ios_settimeoutinterval(soap, 45.0);
        
        double result;
        
        // Call Web service operation add
        int status = soap_call_ns2__add(soap, NULL, NULL,x, y, result);
        
        soap_free_temp(soap); // Cleanup temporary resources
        
        // Check soap response status
        if (status == SOAP_OK) {
            
            NSString *resultString;
            NSString *titleString;
            
            resultString = [NSString stringWithFormat:@"%f",result];
            titleString = [NSString stringWithFormat:@"%f + %f =",x, y];
          
            // Show the result in an alert
            UIAlertController * alert = [UIAlertController
                                         alertControllerWithTitle:titleString
                                         message:resultString
                                         preferredStyle:UIAlertControllerStyleAlert];
            UIAlertAction* cancelButton = [UIAlertAction
                                           actionWithTitle:@"OK"
                                           style:UIAlertActionStyleDefault
                                           handler:^(UIAlertAction * action){}];
            
            [alert addAction: cancelButton];
            [self presentViewController:alert animated:YES completion:nil];
        } else {
            // Caveat: Better to extract the error message and show it using an alert
            soap_print_fault(soap,stderr); // Print soap error in console
        }
    }
    
    soap_destroy(soap);    
    soap_end(soap);
    soap_free(soap);

}

@end
