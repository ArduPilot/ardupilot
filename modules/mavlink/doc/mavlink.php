<?php

// Requires the installation of php5-xsl
// e.g. on Debian/Ubuntu: sudo apt-get install php5-xsl

// Load the file from the repository / server.
// Update this URL if the file location changes

$xml_file_name = "https://raw.github.com/mavlink/mavlink/master/message_definitions/v1.0/common.xml";

// Load the XSL transformation file from the repository / server.
// This file can be updated by any client to adjust the table

$xsl_file_name= "https://raw.github.com/mavlink/mavlink/master/doc/mavlink_to_html_table.xsl";

// Load data XML file
$xml = file_get_contents($xml_file_name);
$xml_doc = new DomDocument;
$xml_doc->loadXML($xml);

// Load stylesheet XSL file
$xsl = file_get_contents($xsl_file_name);
$xsl_doc = new DomDocument;
$xsl_doc->loadXML($xsl);

$xsltproc = new XsltProcessor();
$xsltproc->importStylesheet($xsl_doc);
?>

<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01//EN">
<html>
<head>
	<title>MAVLINK Common Message set specifications</title>
	<link rel="stylesheet" type="text/css" href="mavlink.css">
</head>
<body>
<h1>MAVLINK Common Message Set</h1>

<p>
These messages define the common message set, which is the reference message set implemented by most ground control stations and autopilots.
</p>

<?php
// process the files and write the output to $out_file
if ($html = $xsltproc->transformToXML($xml_doc))
{
  echo $html;
}
else
{
  trigger_error("XSL transformation failed",E_USER_ERROR);
}

?>

<br />
<br />

<p>
Messages are defined by the <a href="https://raw.github.com/mavlink/mavlink/master/message_definitions/v1.0/common.xml">common.xml</a> file. The C packing/unpacking code is generated from this specification, as well as the HTML documentation in the section above.<br />
<br />
<i>The XML displayed here is updated on every commit and therefore up-to-date.</i>
</p>
</body>
</html>
<?php
//require_once("inc/geshi.php");
//$xml_file_name = "http://github.com/pixhawk/mavlink/raw/master/mavlink_standard_message.xml";
//
//// Load data XML file
//$xml = file_get_contents($xml_file_name);
//
//// Show the current code
//$geshi_xml = new GeSHi($xml, 'xml');
//$display_xml = $geshi_xml->parse_code();
//
//echo $display_xml;

?>