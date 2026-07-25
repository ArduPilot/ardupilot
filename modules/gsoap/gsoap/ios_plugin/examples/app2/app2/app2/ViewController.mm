//
//  ViewController.mm
//

#import "ViewController.h"
#include "soapGeoIPServiceSoapProxy.h"
#import "soapStub.h"
#import "gsoapios.h"
using namespace std;

//@interface ViewController ()

//@end

typedef struct _ns1__GetGeoIP RequestStruct;
typedef struct _ns1__GetGeoIPResponse ResponseStruct;


@implementation ViewController

@synthesize IPAddress;

- (IBAction)buttonPressed:(id)sender {
    RequestStruct ip;
    ResponseStruct response;
    
    //creates proxy
    GeoIPServiceSoapProxy service(SOAP_IO_DEFAULT|SOAP_IO_KEEPALIVE|SOAP_XML_INDENT|SOAP_XML_STRICT);
    soap_init(service.soap);
    
    //sets IPAddress from input
    std::string ipAdd = std::string((char *)[IPAddress.text UTF8String]);
    ip.IPAddress = &ipAdd;
    
    // ----- register plugin for callbacks ------
    soap_register_plugin(service.soap, soap_ios);
    
    // Optional: timeout internal, the default is 60.0 seconds
    soap_ios_settimeoutinterval(service.soap, 30.0);
    
    //call web service
    int status = service.GetGeoIP(&ip, response);
    
    string* result;
    string err_msg = "Invalid IP address";
    
    if ( status == SOAP_OK) {
        NSString *soapResult;
        NSString *titleMsg;
        
        if(response.GetGeoIPResult && response.GetGeoIPResult->CountryName) {
            result = response.GetGeoIPResult->CountryName;
        }
        else {
            result = &err_msg;
        }
        
        soapResult = [NSString stringWithUTF8String:result->c_str()];
        titleMsg = [NSString stringWithFormat: @"%@", @"Country Found"];

        //show result as an alert
        UIAlertController * alert = [UIAlertController
                                     alertControllerWithTitle:titleMsg
                                     message:soapResult
                                     preferredStyle:UIAlertControllerStyleAlert];
        UIAlertAction* cancelButton = [UIAlertAction
                                       actionWithTitle:@"OK"
                                       style:UIAlertActionStyleDefault
                                       handler:^(UIAlertAction * action){}];
        
        [alert addAction: cancelButton];
        [self presentViewController:alert animated:YES completion:nil];
        
    } else {
        service.soap_stream_fault(std::cerr);
    }
    
    service.destroy();
}

// Override to allow orientations other than the default portrait orientation.
- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation {
    // Return YES for supported orientations
    return (interfaceOrientation == UIInterfaceOrientationPortrait ||
            interfaceOrientation == UIInterfaceOrientationPortraitUpsideDown ||
            interfaceOrientation == UIInterfaceOrientationLandscapeLeft ||
            interfaceOrientation == UIInterfaceOrientationLandscapeRight);
}


- (void)didReceiveMemoryWarning {
    // Releases the view if it doesn't have a superview.
    [super didReceiveMemoryWarning];
    
    // Release any cached data, images, etc that aren't in use.
}

- (void)viewDidUnload {
    // Release any retained subviews of the main view.
    // e.g. self.myOutlet = nil;
}


- (void)dealloc {
    //    [ipAddress release];
    //    [super dealloc];
}

@end
