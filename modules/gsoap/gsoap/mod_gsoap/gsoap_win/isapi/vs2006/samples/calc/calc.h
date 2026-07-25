//gsoap ns service name: calc
//gsoap ns service namespace: http://websrv.cs.fsu.edu/~engelen/calc.wsdl
//gsoap ns service location: http://websrv.cs.fsu.edu/~engelen
//gsoap ns service executable: calc.cgi
//gsoap ns schema namespace: urn:calc
int ns__add(double a, double b, double *result);
int ns__sub(double a, double b, double *result);
int ns__mul(double a, double b, double *result);
int ns__div(double a, double b, double *result);
int ns__pow(double a, double b, double *result);
