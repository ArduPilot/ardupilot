<?xml version="1.0"?>

<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:template match="//messages">
   <h4>MAVLink Messages</h4>
   <xsl:apply-templates />
</xsl:template>

<xsl:template match="//message">
   <h3 class="mavlink_message_name"><xsl:value-of select="@name" /> (#<xsl:value-of select="@id" />)</h3>
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
   <td class="mavlink_name"><xsl:value-of select="@name" /></td>
   <td class="mavlink_type"><xsl:value-of select="@type" /></td>
   <td class="mavlink_comment"><xsl:value-of select="." /></td>
   </tr>
</xsl:template>

</xsl:stylesheet>
