<?xml version="1.0"?>

<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:template match="//include">
   <h1>MAVLink Include Files</h1>
   <p><strong><em>Including files: </em><xsl:value-of select="." /></strong></p>
</xsl:template>

<xsl:template match="//enums">
   <h1>MAVLink Type Enumerations</h1>
   <xsl:apply-templates />
</xsl:template>

<xsl:template match="//messages">
   <h1>MAVLink Messages</h1>
   <xsl:apply-templates />
</xsl:template>

<xsl:template match="//message">
  <a>
    <xsl:attribute name="name">
      <xsl:value-of select="@name"/>
    </xsl:attribute>
  </a>
  <h3 class="mavlink_message_name"><xsl:value-of select="@name" /> (
   <a>
    <xsl:attribute name="href">
      #<xsl:value-of select="@name"/>
    </xsl:attribute>
   #<xsl:value-of select="@id" />
   </a>
   )</h3>
   <p class="description"><xsl:value-of select="description" /></p>
   <table class="sortable">
   <thead>
   <tr>
     <th class="mavlink_field_header">Field Name</th>
     <th class="mavlink_field_header">Type</th>
     <th class="mavlink_field_header">Description</th>
   </tr>
   </thead>
   <tbody>
   <xsl:apply-templates select="field" />
  </tbody>
  </table>
</xsl:template>

<xsl:template match="//field">
   <tr class="mavlink_field">
   <td class="mavlink_name" valign="top"><xsl:value-of select="@name" /></td>
   <td class="mavlink_type" valign="top"><xsl:value-of select="@type" /></td>
   <td class="mavlink_comment"><xsl:value-of select="." /></td>
   </tr>
</xsl:template>

<xsl:template match="//version">
   <h2 style="color:red;">MAVLink Documentation</h2>
   <p>
   The <a href="https://mavlink.io/en/messages/common.html">Official MAVLink message documentation</a> contains additional information, including field units and enum values.
   </p>
   
   <h2>MAVLink Protocol Version</h2>
   <p>The current MAVLink version is 2.<xsl:value-of select="." />. The minor version numbers (after the dot) range from 1-255. </p>
</xsl:template>

<xsl:template match="//enum">
   <a>
    <xsl:attribute name="name">
      ENUM_<xsl:value-of select="@name"/>
    </xsl:attribute>
  </a>
   <h3 class="mavlink_message_name"><xsl:value-of select="@name" /></h3>

   <p class="description"><xsl:value-of select="description" /></p>
   <table class="sortable">
   <thead>
   <tr>
     <th class="mavlink_field_header">Value</th>
     <th class="mavlink_field_header">Field Name</th>
     <th class="mavlink_field_header">Description</th>
   </tr>
   </thead>
   <tbody>
   <xsl:apply-templates select="entry" />
  </tbody>
  </table>
</xsl:template>

<xsl:template match="//entry">
   <tr class="mavlink_field" id="{@name}">
   <td class="mavlink_type" valign="top"><xsl:value-of select="@value" /></td>
   <td class="mavlink_name" valign="top"><xsl:value-of select="@name" /></td>
   <td class="mavlink_comment"><xsl:value-of select="description" /></td>
   </tr>
   <tr>
     <td></td>
   	 <xsl:apply-templates select="param" />
   </tr>
   <tr>
    <td colspan="3"><br /></td>
   </tr>
</xsl:template>

<xsl:template match="//param">
   <tr>
   <td></td>
   <td class="mavlink_mission_param" valign="top">Mission Param #<xsl:value-of select="@index" /></td>
   <td class="mavlink_comment"><xsl:value-of select="." /></td>
   </tr>
</xsl:template>


</xsl:stylesheet>
