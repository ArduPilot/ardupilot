proc contents data=sashelp.retail varnum;
run;

proc univariate data=sashelp.retail;
    var sales;
run;


%let rootFldr = C:\waf\demos\sas\;
filename mydata "&rootFldr.data\mydata.csv";

data mydata;
    infile mydata delimiter=',' dsd firstobs=2;
    input
        id
        var1
        var2 $
    ;
run;

proc print data=mydata;
run;

