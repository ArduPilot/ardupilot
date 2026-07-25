//gsoap ns service name:	quote
//gsoap ns service style:	rpc
//gsoap ns service encoding:	encoded
//gsoap ns service namespace:	urn:xmethods-delayed-quotes
//gsoap ns service location:	http://64.124.140.30:9090/soap

//gsoap ns service method-action: getQuote ""

int ns__getQuote(char *symbol, float *Result);
