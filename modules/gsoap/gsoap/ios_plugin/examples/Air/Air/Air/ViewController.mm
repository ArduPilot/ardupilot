//
//  ViewController.mm
// 

#import "ViewController.h"
#include "soapairportSoapProxy.h"
#import "soapStub.h"
#import "gsoapios.h"
#include <sstream>

@interface ViewController ()

@end

@implementation ViewController
@synthesize country_name;
@synthesize showResults;

- (IBAction)buttonPressed:(id)sender {
    _ns1__GetAirportInformationByCountry sending;
    _ns1__GetAirportInformationByCountryResponse receiving;
    std::string result = "";                                    //stores resulting data
    bool webserviceresult = true;                               //keeps track of if the web service returns a readable result
    
    airportSoapProxy service;
    soap_init(service.soap);
    
    // ----- register plugin for callbacks ------
    soap_register_plugin(service.soap, soap_ios);
    
    std::string countryname = std::string((char*)[country_name.text UTF8String]);
    sending.country = &countryname;

    std::stringstream xmlmessage;

    if(service.GetAirportInformationByCountry(&sending,receiving)==SOAP_OK)
            xmlmessage << *(receiving.GetAirportInformationByCountryResult);
 
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_DOM_TREE);
    ctx->double_format = "%lG";
 
    xsd__anyType dom(ctx);
    xmlmessage >> dom;
    if (dom.soap->error)
        webserviceresult = false;
    xsd__anyType *elt;

#define USE_ATT(path, text) std::cout << path << " = " << text << std::endl
#define USE_ELT(path, text) std::cout << path << " = " << text << std::endl

    for (xsd__anyType *it = dom.elt_get("Table"); it; it = it->get_next())
    {
        xsd__anyType& dom_Table = *it;
        if ((elt = dom_Table.elt_get("AirportCode")))
        {
            xsd__anyType& dom_Table_AirportCode = *elt;
            if (dom_Table_AirportCode.get_text())
            {
                result += "\n\nAirport Code = ";
                result += dom_Table_AirportCode.get_text();
            }

        }
        if ((elt = dom_Table.elt_get("CityOrAirportName")))
        {
            xsd__anyType& dom_Table_CityOrAirportName = *elt;
            if (dom_Table_CityOrAirportName.get_text())
            {
                result += "\nCity or Airport Name = ";
                result += dom_Table_CityOrAirportName.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("Country")))
        {
            xsd__anyType& dom_Table_Country = *elt;
            if (dom_Table_Country.get_text())
            {
                result += "\nCountry = ";
                result += dom_Table_Country.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("CountryAbbrviation")))
        {
            xsd__anyType& dom_Table_CountryAbbrviation = *elt;
            if (dom_Table_CountryAbbrviation.get_text())
            {
                result += "\nCountry Abbreviation = ";
                result += dom_Table_CountryAbbrviation.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("CountryCode")))
        {
            xsd__anyType& dom_Table_CountryCode = *elt;
            if (dom_Table_CountryCode.get_text())
            {
                result += "\nCountry Code = ";
                result += dom_Table_CountryCode.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("GMTOffset")))
        {
            xsd__anyType& dom_Table_GMTOffset = *elt;
            if (dom_Table_GMTOffset.get_text())
            {
                result += "\nGMT Offset = ";
                result += dom_Table_GMTOffset.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("RunwayLengthFeet")))
        {
            xsd__anyType& dom_Table_RunwayLengthFeet = *elt;
            if (dom_Table_RunwayLengthFeet.get_text())
            {
                result += "\nRunway Length (feet) = ";
                result += dom_Table_RunwayLengthFeet.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("RunwayElevationFeet")))
        {
            xsd__anyType& dom_Table_RunwayElevationFeet = *elt;
            if (dom_Table_RunwayElevationFeet.get_text())
            {
                result += "\nRunway Elevation (feet) = ";
                result += dom_Table_RunwayElevationFeet.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("LatitudeDegree")))
        {
            xsd__anyType& dom_Table_LatitudeDegree = *elt;
            if (dom_Table_LatitudeDegree.get_text())
            {
                result += "\nLatitude Degree = ";
                result += dom_Table_LatitudeDegree.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("LatitudeMinute")))
        {
            xsd__anyType& dom_Table_LatitudeMinute = *elt;
            if (dom_Table_LatitudeMinute.get_text())
            {
                result += "\nLatitude Minute = ";
                result += dom_Table_LatitudeMinute.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("LatitudeSecond")))
        {
            xsd__anyType& dom_Table_LatitudeSecond = *elt;
            if (dom_Table_LatitudeSecond.get_text())
            {
                result += "\nLatitude Second = ";
                result += dom_Table_LatitudeSecond.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("LatitudeNpeerS")))
        {
            xsd__anyType& dom_Table_LatitudeNpeerS = *elt;
            if (dom_Table_LatitudeNpeerS.get_text())
            {
                result += "\nLatitude N or S = ";
                result += dom_Table_LatitudeNpeerS.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("LongitudeDegree")))
        {
            xsd__anyType& dom_Table_LongitudeDegree = *elt;
            if (dom_Table_LongitudeDegree.get_text())
            {
                result += "\nLongitude Degree = ";
                result += dom_Table_LongitudeDegree.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("LongitudeMinute")))
        {
            xsd__anyType& dom_Table_LongitudeMinute = *elt;
            if (dom_Table_LongitudeMinute.get_text())
            {
                result += "\nLongitude Minute = ";
                result += dom_Table_LongitudeMinute.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("LongitudeSeconds")))
        {
            xsd__anyType& dom_Table_LongitudeSeconds = *elt;
            if (dom_Table_LongitudeSeconds.get_text())
            {
                result += "\nLongitude Second = ";
                result += dom_Table_LongitudeSeconds.get_text();
            }
        }
        if ((elt = dom_Table.elt_get("LongitudeEperW")))
        {
            xsd__anyType& dom_Table_LongitudeEperW = *elt;
            if (dom_Table_LongitudeEperW.get_text())
            {
                result += "\nLongitude E or W = ";
                result += dom_Table_LongitudeEperW.get_text();
            }
        }
    }

    NSString *airPorts = [NSString stringWithUTF8String:result.c_str()];
    if(result == "" || !webserviceresult)
        airPorts = [NSString stringWithFormat: @"%@", @"Error: no know information. Try a different country."];
    
    showResults.editable = NO;
    showResults.showsVerticalScrollIndicator = YES;
    showResults.text = airPorts;
    
    soap_destroy(ctx); // delete objects
    soap_end(ctx);     // delete DOM data
    soap_free(ctx);    // free context
    service.destroy();
}

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
