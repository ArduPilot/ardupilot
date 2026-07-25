<?xml version="1.0"?>

<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:template match="//include">
   <p><strong>MAVLink Include Files: </strong> <a><xsl:attribute name="href"><xsl:value-of select="."/>.md.unlikely</xsl:attribute><xsl:value-of select="." /></a> </p>
</xsl:template>

<xsl:template match="//enums">
   <h2 id="enums">MAVLink Type Enumerations</h2>
   <xsl:apply-templates select="enum[@name!='MAV_CMD']" />

   <a id="MAV_CMD"></a>
   <h2 id="mav_commands">MAVLink Commands (MAV_CMD)</h2>
   <blockquote class="alert alert-info clearfix"><strong class="fa fa-2x fa-edit"></strong><p>MAVLink commands (MAV_CMD) and messages are different! These commands define the values of up to 7 parameters that are packaged INSIDE specific messages used in the Mission Protocol and Command Protocol. Use commands for actions in missions or if you need acknowledgment and/or retry logic from a request. Otherwise use messages.</p></blockquote>
   <xsl:apply-templates select="enum[@name='MAV_CMD']" mode="params"/>

</xsl:template>



<xsl:template match="//messages">
   <h2 id="messages">MAVLink Messages</h2>
   <xsl:apply-templates />
</xsl:template>

<xsl:template match="//message">
  <h3> <!-- mavlink_message_name -->
   <xsl:attribute name="id"><xsl:value-of select="@name"/></xsl:attribute>
   <xsl:value-of select="@name" /> (
   <a>
    <xsl:attribute name="href">#<xsl:value-of select="@name"/></xsl:attribute>
    #<xsl:value-of select="@id" />
   </a>
  )</h3>
   <xsl:apply-templates select="wip" />
   <xsl:apply-templates select="deprecated" />
   <p> <!-- description --><a href="#messages">[Message]</a>
     <xsl:if test='@id > 255'><strong>(MAVLink 2) </strong></xsl:if>
     <xsl:value-of select="description" /></p>
   <table class="sortable">
   <thead>
   <tr> <!-- mavlink_field_header -->
     <th>Field Name</th>
     <th>Type</th>

     <xsl:if test='*/@units'>
      <th>Units</th>
     </xsl:if>
     
     <xsl:if test='*/@enum'>
      <th>Values</th>
     </xsl:if>

     <th>Description</th>
   </tr>
   </thead>
   <tbody>
   <xsl:apply-templates select="field" /> 
  </tbody>
  </table>
</xsl:template>


<xsl:template match="//field">
   <tr> <!-- mavlink_field -->
   <xsl:choose>
     <xsl:when test="preceding-sibling::extensions">
       <td style="color:blue;"><xsl:value-of select="@name" />&#160;<a href="#mav2_extension_field" title="MAVLink2 extension field">**</a></td> <!-- mavlink_name -->
     </xsl:when>
     <xsl:otherwise>
       <td><xsl:value-of select="@name" /></td> <!-- mavlink_name -->
     </xsl:otherwise>
   </xsl:choose>
  
   <td><xsl:value-of select="@type" /></td> <!-- mavlink_type -->
   
   <xsl:if test='../*/@units'>
     <td><xsl:value-of select="@units" /></td> <!-- mavlink_units -->
   </xsl:if>
   
   <xsl:if test='../*/@enum'>
   <td><xsl:if test='@enum'>
      <a><xsl:attribute name="href">#<xsl:value-of select="@enum" /></xsl:attribute><xsl:value-of select="@enum" /></a>
      </xsl:if>
   </td> <!-- mavlink_value -->
   </xsl:if>
     
   <td><xsl:value-of select="." /></td> <!-- mavlink_comment -->
   </tr>
</xsl:template>

<xsl:template match="//version">
   <h2>MAVLink Protocol Version</h2>
   <p>The current MAVLink version is 2.<xsl:value-of select="." />. The minor version numbers (after the dot) range from 1-255. </p>
</xsl:template>

<xsl:template match="//dialect">
   <p>This file has protocol dialect: <xsl:value-of select="." />.</p>
</xsl:template>


<xsl:template match="//enum">
   <h3> <!-- mavlink_enum_name -->
     <xsl:attribute name="id"><xsl:value-of select="@name"/></xsl:attribute>
     <xsl:value-of select="@name" /></h3>
   <xsl:apply-templates select="deprecated" />  
   <p><a href="#enums">[Enum]</a><xsl:value-of select="description" /></p> <!-- description -->
   <table class="sortable">
   <thead>
   <tr> <!-- mavlink_field_header -->
     <th>Value</th>
     <th>Field Name</th>
     <th>Description</th>
   </tr>
   </thead>
   <tbody>
   <xsl:apply-templates select="entry" />
  </tbody>
  </table>
</xsl:template>


<xsl:template match="//enum" mode="params">
   <p><xsl:value-of select="description" /> </p>
   <xsl:apply-templates select="entry" mode="params" />
</xsl:template>


<xsl:template match="//entry" mode="params">
   <h3 id="{@name}"><xsl:value-of select="@name" /> (<a><xsl:attribute name="href">#<xsl:value-of select="@name"/></xsl:attribute><xsl:value-of select="@value" /></a>)</h3>
      <xsl:apply-templates select="deprecated" />
      <xsl:apply-templates select="wip" />
      <p><a href="#mav_commands">[Command]</a><xsl:value-of select="description" /> </p> <!-- mavlink_comment -->


   <table class="sortable">
   <thead>
   <tr> <!-- mavlink_field_header -->
      <th>Param (:Label)</th>
      <th>Description</th>

      <xsl:if test='*/@enum or */@minValue or */@maxValue or */@increment'>
        <th>Values</th>
      </xsl:if>

     <xsl:if test='*/@units'>
       <th>Units</th>
     </xsl:if>

   </tr>
   </thead>
   <tbody>
    <xsl:apply-templates select="param" mode="params" /> 
   </tbody>
  </table>

</xsl:template>


<xsl:template match="//entry">
   <tr id="{@name}"> <!-- mavlink_field -->
   <td><xsl:value-of select="@value" /></td>  <!-- mavlink_type -->
   <td>
      <a><xsl:attribute name="href">#<xsl:value-of select="@name"/></xsl:attribute><xsl:value-of select="@name" /></a> 
      <xsl:apply-templates select="deprecated" />
      <xsl:apply-templates select="wip" />
   </td> <!-- mavlink_name -->
   <td><xsl:value-of select="description" /></td> <!-- mavlink_comment -->
   </tr>


<xsl:if test='param'>
   <tr>
     <td></td>
     <xsl:apply-templates select="param" />
   </tr>
   <tr>
    <td colspan="3"><br /></td>
   </tr>
</xsl:if>
</xsl:template>



<xsl:template match="//param" mode="params">
    <tr>
        <td><xsl:value-of select="@index" /> 
        <xsl:if test='@label'>: <xsl:value-of select="@label" /></xsl:if>
        </td> <!-- mission_param -->

        <td><xsl:if test='@reserved = "true"'>Reserved (set to <xsl:if test='@default'><xsl:value-of select="@default" /></xsl:if><xsl:if test='not(@default)'>0</xsl:if>)</xsl:if><xsl:value-of select="." />
            <xsl:if test='@decimalPlaces'><br /><strong>GCS display settings:</strong>
            <xsl:if test='@label'><em>Label:</em> <xsl:value-of select="@label" />, </xsl:if>
            <xsl:if test='@decimalPlaces'><em>decimalPlaces:</em> <xsl:value-of select="@decimalPlaces" /></xsl:if>
         </xsl:if>
        </td>


   <xsl:if test='../*/@enum or ../*/@minValue or ../*/@maxValue or ../*/@increment'>
     <td>
      <xsl:choose>
         <xsl:when test="@enum">
           <a><xsl:attribute name="href">#<xsl:value-of select="@enum" /></xsl:attribute><xsl:value-of select="@enum" /></a>
         </xsl:when>
         <xsl:when test="@minValue or @maxValue or @increment ">
           <xsl:if test='@minValue'><em>min:</em><xsl:value-of select="@minValue" /><xsl:text>xxx_space_xxx</xsl:text></xsl:if>
           <xsl:if test='@maxValue'><em>max:</em><xsl:value-of select="@maxValue" /><xsl:text>xxx_space_xxx</xsl:text></xsl:if>
           <xsl:if test='@increment'><em>increment:</em><xsl:value-of select="@increment" /></xsl:if>
         </xsl:when>
      </xsl:choose>
  </td>
   </xsl:if>
      
   <xsl:if test='../*/@units'>
     <td><xsl:value-of select="@units" /></td> <!-- mavlink_units -->
   </xsl:if>
       
   </tr>
</xsl:template>



<xsl:template match="//param">
   <tr>
   <td></td>
   <td>Param #<xsl:value-of select="@index" /></td> <!-- mission_param -->
   <td>
       <xsl:value-of select="." />

       <xsl:choose>
         <xsl:when test="@enum">
            <br /><strong>Possible values:</strong> <xsl:value-of select="@enum" />
         </xsl:when>
         <xsl:when test="@minValue or @maxValue or @increment or @units">
           <br /><strong>Values:</strong>
           <xsl:if test='@units'><em>units:</em> <xsl:value-of select="@minValue" />, </xsl:if>
           <xsl:if test='@minValue'><em>min:</em><xsl:value-of select="@minValue" />, </xsl:if>
           <xsl:if test='@maxValue'><em>max:</em><xsl:value-of select="@maxValue" />, </xsl:if>
           <xsl:if test='@increment'><em>increment:</em><xsl:value-of select="@increment" /></xsl:if>
         </xsl:when>
       </xsl:choose>

       <xsl:if test='@label or @decimalPlaces'><br /><strong>GCS display settings:</strong>
           <xsl:if test='@label'><em>Label:</em> <xsl:value-of select="@label" />, </xsl:if>
           <xsl:if test='@decimalPlaces'><em>decimalPlaces:</em> <xsl:value-of select="@decimalPlaces" /></xsl:if>
       </xsl:if>


   </td> <!-- mavlink_comment -->
   </tr>
</xsl:template>

<xsl:template match="//wip">
  <p style="color:red"><strong>WORK IN PROGRESS:</strong><xsl:text>xxx_space_xxx</xsl:text>Do not use in stable production environments (it may change).</p>
</xsl:template>

<xsl:template match="//deprecated">
  <p style="color:red"><strong>DEPRECATED:</strong><xsl:text>xxx_space_xxx</xsl:text>Replaced by <xsl:value-of select="@replaced_by" /> (<xsl:value-of select="@since" />).


  <xsl:if test='.'>
    <xsl:value-of select="." />
  </xsl:if>
</p>
</xsl:template>



</xsl:stylesheet>
