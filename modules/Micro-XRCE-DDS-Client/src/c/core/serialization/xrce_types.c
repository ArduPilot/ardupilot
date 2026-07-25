#include <uxr/client/core/type/xrce_types.h>
#include <string.h>

//==================================================================
//                             PUBLIC
//==================================================================
bool uxr_serialize_Time_t(
        ucdrBuffer* buffer,
        const Time_t* input)
{
    bool ret = true;
    ret &= ucdr_serialize_int32_t(buffer, input->seconds);
    ret &= ucdr_serialize_uint32_t(buffer, input->nanoseconds);
    return ret;
}

bool uxr_deserialize_Time_t(
        ucdrBuffer* buffer,
        Time_t* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_int32_t(buffer, &output->seconds);
    ret &= ucdr_deserialize_uint32_t(buffer, &output->nanoseconds);
    return ret;
}

bool uxr_serialize_BinarySequence_t(
        ucdrBuffer* buffer,
        const BinarySequence_t* input)
{
    return ucdr_serialize_sequence_uint8_t(buffer, input->data, input->size);
}

bool uxr_deserialize_BinarySequence_t(
        ucdrBuffer* buffer,
        BinarySequence_t* output)
{
    return ucdr_deserialize_sequence_uint8_t(buffer, output->data, UXR_BINARY_SEQUENCE_MAX, &output->size);
}

bool uxr_serialize_StringSequence_t(
        ucdrBuffer* buffer,
        const StringSequence_t* input)
{
    bool ret = ucdr_serialize_uint32_t(buffer, input->size);
    for (uint32_t i = 0; i < input->size && ret; i++)
    {
        ret = ucdr_serialize_string(buffer, input->data[i]);
    }
    return ret;
}

bool uxr_deserialize_StringSequence_t(
        ucdrBuffer* buffer,
        StringSequence_t* output)
{
    bool ret = ucdr_deserialize_uint32_t(buffer, &output->size);
    if (output->size > UXR_STRING_SEQUENCE_MAX)
    {
        buffer->error = true;
        ret = false;
    }
    else
    {
        for (uint32_t i = 0; i < output->size && ret; i++)
        {
            ret = ucdr_deserialize_string(buffer, output->data[i], UXR_STRING_SIZE_MAX);
        }
    }
    return ret;
}

bool uxr_serialize_ClientKey(
        ucdrBuffer* buffer,
        const ClientKey* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->data, 4);
    return ret;
}

bool uxr_deserialize_ClientKey(
        ucdrBuffer* buffer,
        ClientKey* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->data, 4);
    return ret;
}

bool uxr_serialize_ObjectId(
        ucdrBuffer* buffer,
        const ObjectId* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->data, 2);
    return ret;
}

bool uxr_deserialize_ObjectId(
        ucdrBuffer* buffer,
        ObjectId* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->data, 2);
    return ret;
}

bool uxr_serialize_ObjectPrefix(
        ucdrBuffer* buffer,
        const ObjectPrefix* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->data, 2);
    return ret;
}

bool uxr_deserialize_ObjectPrefix(
        ucdrBuffer* buffer,
        ObjectPrefix* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->data, 2);
    return ret;
}

bool uxr_serialize_XrceCookie(
        ucdrBuffer* buffer,
        const XrceCookie* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->data, 4);
    return ret;
}

bool uxr_deserialize_XrceCookie(
        ucdrBuffer* buffer,
        XrceCookie* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->data, 4);
    return ret;
}

bool uxr_serialize_XrceVersion(
        ucdrBuffer* buffer,
        const XrceVersion* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->data, 2);
    return ret;
}

bool uxr_deserialize_XrceVersion(
        ucdrBuffer* buffer,
        XrceVersion* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->data, 2);
    return ret;
}

bool uxr_serialize_XrceVendorId(
        ucdrBuffer* buffer,
        const XrceVendorId* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->data, 2);
    return ret;
}

bool uxr_deserialize_XrceVendorId(
        ucdrBuffer* buffer,
        XrceVendorId* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->data, 2);
    return ret;
}

bool uxr_serialize_TransportLocatorSmall(
        ucdrBuffer* buffer,
        const TransportLocatorSmall* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->address, 2);
    ret &= ucdr_serialize_uint8_t(buffer, input->locator_port);
    return ret;
}

bool uxr_deserialize_TransportLocatorSmall(
        ucdrBuffer* buffer,
        TransportLocatorSmall* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->address, 2);
    ret &= ucdr_deserialize_uint8_t(buffer, &output->locator_port);
    return ret;
}

bool uxr_serialize_TransportLocatorMedium(
        ucdrBuffer* buffer,
        const TransportLocatorMedium* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->address, 4);
    ret &= ucdr_serialize_uint16_t(buffer, input->locator_port);
    return ret;
}

bool uxr_deserialize_TransportLocatorMedium(
        ucdrBuffer* buffer,
        TransportLocatorMedium* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->address, 4);
    ret &= ucdr_deserialize_uint16_t(buffer, &output->locator_port);
    return ret;
}

bool uxr_serialize_TransportLocatorLarge(
        ucdrBuffer* buffer,
        const TransportLocatorLarge* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->address, 16);
    ret &= ucdr_serialize_uint32_t(buffer, input->locator_port);
    return ret;
}

bool uxr_deserialize_TransportLocatorLarge(
        ucdrBuffer* buffer,
        TransportLocatorLarge* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->address, 16);
    ret &= ucdr_deserialize_uint32_t(buffer, &output->locator_port);
    return ret;
}

bool uxr_serialize_TransportLocatorString(
        ucdrBuffer* buffer,
        const TransportLocatorString* input)
{
    bool ret = true;
    ret &= ucdr_serialize_string(buffer, input->value);
    return ret;
}

bool uxr_deserialize_TransportLocatorString(
        ucdrBuffer* buffer,
        TransportLocatorString* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_string(buffer, output->value, UXR_STRING_SIZE_MAX);
    return ret;
}

bool uxr_serialize_TransportLocator(
        ucdrBuffer* buffer,
        const TransportLocator* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->format);
    if (ret)
    {
        switch (input->format)
        {
            case ADDRESS_FORMAT_SMALL:
                ret &= uxr_serialize_TransportLocatorSmall(buffer, &input->_.small_locator);
                break;
            case ADDRESS_FORMAT_MEDIUM:
                ret &= uxr_serialize_TransportLocatorMedium(buffer, &input->_.medium_locator);
                break;
            case ADDRESS_FORMAT_LARGE:
                ret &= uxr_serialize_TransportLocatorLarge(buffer, &input->_.large_locator);
                break;
            case ADDRESS_FORMAT_STRING:
                ret &= uxr_serialize_TransportLocatorString(buffer, &input->_.string_locator);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_deserialize_TransportLocator(
        ucdrBuffer* buffer,
        TransportLocator* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->format);
    if (ret)
    {
        switch (output->format)
        {
            case ADDRESS_FORMAT_SMALL:
                ret &= uxr_deserialize_TransportLocatorSmall(buffer, &output->_.small_locator);
                break;
            case ADDRESS_FORMAT_MEDIUM:
                ret &= uxr_deserialize_TransportLocatorMedium(buffer, &output->_.medium_locator);
                break;
            case ADDRESS_FORMAT_LARGE:
                ret &= uxr_deserialize_TransportLocatorLarge(buffer, &output->_.large_locator);
                break;
            case ADDRESS_FORMAT_STRING:
                ret &= uxr_deserialize_TransportLocatorString(buffer, &output->_.string_locator);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_serialize_TransportLocatorSeq(
        ucdrBuffer* buffer,
        const TransportLocatorSeq* input)
{
    bool ret = ucdr_serialize_uint32_t(buffer, input->size);
    for (uint32_t i = 0; i < input->size && ret; i++)
    {
        ret = uxr_serialize_TransportLocator(buffer, &input->data[i]);
    }
    return ret;
}

bool uxr_deserialize_TransportLocatorSeq(
        ucdrBuffer* buffer,
        TransportLocatorSeq* output)
{
    bool ret = ucdr_deserialize_uint32_t(buffer, &output->size);
    if (output->size > UXR_TRANSPORT_LOCATOR_SEQUENCE_MAX)
    {
        buffer->error = true;
        ret = false;
    }
    else
    {
        for (uint32_t i = 0; i < output->size && ret; i++)
        {
            ret = uxr_deserialize_TransportLocator(buffer, &output->data[i]);
        }
    }
    return ret;
}

bool uxr_serialize_Property(
        ucdrBuffer* buffer,
        const Property* input)
{
    bool ret = true;
    ret &= ucdr_serialize_string(buffer, input->name);
    ret &= ucdr_serialize_string(buffer, input->value);
    return ret;
}

bool uxr_deserialize_Property(
        ucdrBuffer* buffer,
        Property* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_string(buffer, output->name, UXR_STRING_SIZE_MAX);
    ret &= ucdr_deserialize_string(buffer, output->value, UXR_STRING_SIZE_MAX);
    return ret;
}

bool uxr_serialize_PropertySeq(
        ucdrBuffer* buffer,
        const PropertySeq* input)
{
    bool ret = ucdr_serialize_uint32_t(buffer, input->size);
    for (uint32_t i = 0; i < input->size && ret; i++)
    {
        ret = uxr_serialize_Property(buffer, input->data + i);
    }
    return ret;
}

bool uxr_deserialize_PropertySeq(
        ucdrBuffer* buffer,
        PropertySeq* output)
{
    bool ret = ucdr_deserialize_uint32_t(buffer, &output->size);

    if (output->size > UXR_PROPERTY_SEQUENCE_MAX)
    {
        buffer->error = true;
        ret = false;
    }
    else
    {
        for (uint32_t i = 0; i < output->size && ret; i++)
        {
            ret = uxr_deserialize_Property(buffer, &output->data[i]);
        }
    }
    return ret;
}

bool uxr_serialize_CLIENT_Representation(
        ucdrBuffer* buffer,
        const CLIENT_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_XrceCookie(buffer, &input->xrce_cookie);
    ret &= uxr_serialize_XrceVersion(buffer, &input->xrce_version);
    ret &= uxr_serialize_XrceVendorId(buffer, &input->xrce_vendor_id);
    ret &= uxr_serialize_ClientKey(buffer, &input->client_key);
    ret &= ucdr_serialize_uint8_t(buffer, input->session_id);
    ret &= ucdr_serialize_bool(buffer, input->optional_properties);
    if (input->optional_properties == true)
    {
        ret &= uxr_serialize_PropertySeq(buffer, &input->properties);
    }
    ret &= ucdr_serialize_uint16_t(buffer, input->mtu);

    return ret;
}

bool uxr_deserialize_CLIENT_Representation(
        ucdrBuffer* buffer,
        CLIENT_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_XrceCookie(buffer, &output->xrce_cookie);
    ret &= uxr_deserialize_XrceVersion(buffer, &output->xrce_version);
    ret &= uxr_deserialize_XrceVendorId(buffer, &output->xrce_vendor_id);
    ret &= uxr_deserialize_ClientKey(buffer, &output->client_key);
    ret &= ucdr_deserialize_uint8_t(buffer, &output->session_id);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_properties);
    if (output->optional_properties == true)
    {
        ret &= uxr_deserialize_PropertySeq(buffer, &output->properties);
    }
    ret &= ucdr_deserialize_uint16_t(buffer, &output->mtu);

    return ret;
}

bool uxr_serialize_AGENT_Representation(
        ucdrBuffer* buffer,
        const AGENT_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_XrceCookie(buffer, &input->xrce_cookie);
    ret &= uxr_serialize_XrceVersion(buffer, &input->xrce_version);
    ret &= uxr_serialize_XrceVendorId(buffer, &input->xrce_vendor_id);
    ret &= ucdr_serialize_bool(buffer, input->optional_properties);
    if (input->optional_properties == true)
    {
        ret &= uxr_serialize_PropertySeq(buffer, &input->properties);
    }

    return ret;
}

bool uxr_deserialize_AGENT_Representation(
        ucdrBuffer* buffer,
        AGENT_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_XrceCookie(buffer, &output->xrce_cookie);
    ret &= uxr_deserialize_XrceVersion(buffer, &output->xrce_version);
    ret &= uxr_deserialize_XrceVendorId(buffer, &output->xrce_vendor_id);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_properties);
    // TODO
    //if(output->optional_properties == true)
    //{
    //        ret &= uxr_deserialize_PropertySeq(buffer, &output->properties);
    //}

    return ret;
}

bool uxr_serialize_OBJK_Representation3Formats(
        ucdrBuffer* buffer,
        const OBJK_Representation3Formats* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->format);
    if (ret)
    {
        switch (input->format)
        {
            case DDS_XRCE_REPRESENTATION_BY_REFERENCE:
                ret &= ucdr_serialize_string(buffer, input->_.object_reference);
                break;
            case DDS_XRCE_REPRESENTATION_AS_XML_STRING:
                ret &= ucdr_serialize_string(buffer, input->_.xml_string_represenatation);
                break;
            case DDS_XRCE_REPRESENTATION_IN_BINARY:
                ret &= uxr_serialize_BinarySequence_t(buffer, &input->_.binary_representation);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_deserialize_OBJK_Representation3Formats(
        ucdrBuffer* buffer,
        OBJK_Representation3Formats* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->format);
    if (ret)
    {
        switch (output->format)
        {
            case DDS_XRCE_REPRESENTATION_BY_REFERENCE:
                ret &= ucdr_deserialize_string(buffer, output->_.object_reference, UXR_STRING_SIZE_MAX);
                break;
            case DDS_XRCE_REPRESENTATION_AS_XML_STRING:
                ret &= ucdr_deserialize_string(buffer, output->_.xml_string_represenatation, UXR_STRING_SIZE_MAX);
                break;
            case DDS_XRCE_REPRESENTATION_IN_BINARY:
                ret &= uxr_deserialize_BinarySequence_t(buffer, &output->_.binary_representation);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_serialize_OBJK_RepresentationRefAndXMLFormats(
        ucdrBuffer* buffer,
        const OBJK_RepresentationRefAndXMLFormats* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->format);
    if (ret)
    {
        switch (input->format)
        {
            case DDS_XRCE_REPRESENTATION_BY_REFERENCE:
                ret &= ucdr_serialize_string(buffer, input->_.object_name);
                break;
            case DDS_XRCE_REPRESENTATION_AS_XML_STRING:
                ret &= ucdr_serialize_string(buffer, input->_.xml_string_represenatation);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_deserialize_OBJK_RepresentationRefAndXMLFormats(
        ucdrBuffer* buffer,
        OBJK_RepresentationRefAndXMLFormats* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->format);
    if (ret)
    {
        switch (output->format)
        {
            case DDS_XRCE_REPRESENTATION_BY_REFERENCE:
                ret &= ucdr_deserialize_string(buffer, output->_.object_name, UXR_STRING_SIZE_MAX);
                break;
            case DDS_XRCE_REPRESENTATION_AS_XML_STRING:
                ret &= ucdr_deserialize_string(buffer, output->_.xml_string_represenatation, UXR_STRING_SIZE_MAX);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_serialize_OBJK_RepresentationBinAndXMLFormats(
        ucdrBuffer* buffer,
        const OBJK_RepresentationBinAndXMLFormats* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->format);
    if (ret)
    {
        switch (input->format)
        {
            case DDS_XRCE_REPRESENTATION_IN_BINARY:
                ret &= uxr_serialize_BinarySequence_t(buffer, &input->_.binary_representation);
                break;
            case DDS_XRCE_REPRESENTATION_AS_XML_STRING:
                ret &= ucdr_serialize_string(buffer, input->_.string_represenatation);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_deserialize_OBJK_RepresentationBinAndXMLFormats(
        ucdrBuffer* buffer,
        OBJK_RepresentationBinAndXMLFormats* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->format);
    if (ret)
    {
        switch (output->format)
        {
            case DDS_XRCE_REPRESENTATION_IN_BINARY:
                ret &= uxr_deserialize_BinarySequence_t(buffer, &output->_.binary_representation);
                break;
            case DDS_XRCE_REPRESENTATION_AS_XML_STRING:
                ret &= ucdr_deserialize_string(buffer, output->_.string_represenatation, UXR_STRING_SIZE_MAX);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_serialize_OBJK_RepresentationRefAndXML_Base(
        ucdrBuffer* buffer,
        const OBJK_RepresentationRefAndXML_Base* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_RepresentationRefAndXMLFormats(buffer, &input->representation);
    return ret;
}

bool uxr_deserialize_OBJK_RepresentationRefAndXML_Base(
        ucdrBuffer* buffer,
        OBJK_RepresentationRefAndXML_Base* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_RepresentationRefAndXMLFormats(buffer, &output->representation);
    return ret;
}

bool uxr_serialize_OBJK_RepresentationBinAndXML_Base(
        ucdrBuffer* buffer,
        const OBJK_RepresentationBinAndXML_Base* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_RepresentationBinAndXMLFormats(buffer, &input->representation);
    return ret;
}

bool uxr_deserialize_OBJK_RepresentationBinAndXML_Base(
        ucdrBuffer* buffer,
        OBJK_RepresentationBinAndXML_Base* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_RepresentationBinAndXMLFormats(buffer, &output->representation);
    return ret;
}

bool uxr_serialize_OBJK_Representation3_Base(
        ucdrBuffer* buffer,
        const OBJK_Representation3_Base* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_Representation3Formats(buffer, &input->representation);
    return ret;
}

bool uxr_deserialize_OBJK_Representation3_Base(
        ucdrBuffer* buffer,
        OBJK_Representation3_Base* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_Representation3Formats(buffer, &output->representation);
    return ret;
}

bool uxr_serialize_OBJK_QOSPROFILE_Representation(
        ucdrBuffer* buffer,
        const OBJK_QOSPROFILE_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_RepresentationRefAndXML_Base(buffer, &input->base);
    return ret;
}

bool uxr_deserialize_OBJK_QOSPROFILE_Representation(
        ucdrBuffer* buffer,
        OBJK_QOSPROFILE_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_RepresentationRefAndXML_Base(buffer, &output->base);
    return ret;
}

bool uxr_serialize_OBJK_TYPE_Representation(
        ucdrBuffer* buffer,
        const OBJK_TYPE_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_RepresentationRefAndXML_Base(buffer, &input->base);
    return ret;
}

bool uxr_deserialize_OBJK_TYPE_Representation(
        ucdrBuffer* buffer,
        OBJK_TYPE_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_RepresentationRefAndXML_Base(buffer, &output->base);
    return ret;
}

bool uxr_serialize_OBJK_DOMAIN_Representation(
        ucdrBuffer* buffer,
        const OBJK_DOMAIN_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_RepresentationRefAndXML_Base(buffer, &input->base);
    return ret;
}

bool uxr_deserialize_OBJK_DOMAIN_Representation(
        ucdrBuffer* buffer,
        OBJK_DOMAIN_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_RepresentationRefAndXML_Base(buffer, &output->base);
    return ret;
}

bool uxr_serialize_OBJK_APPLICATION_Representation(
        ucdrBuffer* buffer,
        const OBJK_APPLICATION_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_RepresentationRefAndXML_Base(buffer, &input->base);
    return ret;
}

bool uxr_deserialize_OBJK_APPLICATION_Representation(
        ucdrBuffer* buffer,
        OBJK_APPLICATION_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_RepresentationRefAndXML_Base(buffer, &output->base);
    return ret;
}

bool uxr_serialize_OBJK_PUBLISHER_Representation(
        ucdrBuffer* buffer,
        const OBJK_PUBLISHER_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_RepresentationBinAndXML_Base(buffer, &input->base);
    ret &= uxr_serialize_ObjectId(buffer, &input->participant_id);
    return ret;
}

bool uxr_deserialize_OBJK_PUBLISHER_Representation(
        ucdrBuffer* buffer,
        OBJK_PUBLISHER_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_RepresentationBinAndXML_Base(buffer, &output->base);
    ret &= uxr_deserialize_ObjectId(buffer, &output->participant_id);
    return ret;
}

bool uxr_serialize_OBJK_SUBSCRIBER_Representation(
        ucdrBuffer* buffer,
        const OBJK_SUBSCRIBER_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_RepresentationBinAndXML_Base(buffer, &input->base);
    ret &= uxr_serialize_ObjectId(buffer, &input->participant_id);
    return ret;
}

bool uxr_deserialize_OBJK_SUBSCRIBER_Representation(
        ucdrBuffer* buffer,
        OBJK_SUBSCRIBER_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_RepresentationBinAndXML_Base(buffer, &output->base);
    ret &= uxr_deserialize_ObjectId(buffer, &output->participant_id);
    return ret;
}

bool uxr_serialize_DATAWRITER_Representation(
        ucdrBuffer* buffer,
        const DATAWRITER_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_Representation3_Base(buffer, &input->base);
    ret &= uxr_serialize_ObjectId(buffer, &input->publisher_id);
    return ret;
}

bool uxr_deserialize_DATAWRITER_Representation(
        ucdrBuffer* buffer,
        DATAWRITER_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_Representation3_Base(buffer, &output->base);
    ret &= uxr_deserialize_ObjectId(buffer, &output->publisher_id);
    return ret;
}

bool uxr_serialize_DATAREADER_Representation(
        ucdrBuffer* buffer,
        const DATAREADER_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_Representation3_Base(buffer, &input->base);
    ret &= uxr_serialize_ObjectId(buffer, &input->subscriber_id);
    return ret;
}

bool uxr_deserialize_DATAREADER_Representation(
        ucdrBuffer* buffer,
        DATAREADER_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_Representation3_Base(buffer, &output->base);
    ret &= uxr_deserialize_ObjectId(buffer, &output->subscriber_id);
    return ret;
}

bool uxr_serialize_OBJK_PARTICIPANT_Representation(
        ucdrBuffer* buffer,
        const OBJK_PARTICIPANT_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_Representation3_Base(buffer, &input->base);
    ret &= ucdr_serialize_int16_t(buffer, input->domain_id);
    return ret;
}

bool uxr_deserialize_OBJK_PARTICIPANT_Representation(
        ucdrBuffer* buffer,
        OBJK_PARTICIPANT_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_Representation3_Base(buffer, &output->base);
    ret &= ucdr_deserialize_int16_t(buffer, &output->domain_id);
    return ret;
}

bool uxr_serialize_OBJK_TOPIC_Representation(
        ucdrBuffer* buffer,
        const OBJK_TOPIC_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_Representation3_Base(buffer, &input->base);
    ret &= uxr_serialize_ObjectId(buffer, &input->participant_id);
    return ret;
}

bool uxr_deserialize_OBJK_TOPIC_Representation(
        ucdrBuffer* buffer,
        OBJK_TOPIC_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_Representation3_Base(buffer, &output->base);
    ret &= uxr_deserialize_ObjectId(buffer, &output->participant_id);
    return ret;
}

bool uxr_serialize_OBJK_REQUESTER_Representation(
        ucdrBuffer* buffer,
        const OBJK_REQUESTER_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_Representation3_Base(buffer, &input->base);
    ret &= uxr_serialize_ObjectId(buffer, &input->participant_id);
    return ret;
}

bool uxr_deserialize_OBJK_REQUESTER_Representation(
        ucdrBuffer* buffer,
        OBJK_REQUESTER_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_Representation3_Base(buffer, &output->base);
    ret &= uxr_deserialize_ObjectId(buffer, &output->participant_id);
    return ret;
}

bool uxr_serialize_OBJK_REPLIER_Representation(
        ucdrBuffer* buffer,
        const OBJK_REPLIER_Representation* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_Representation3_Base(buffer, &input->base);
    ret &= uxr_serialize_ObjectId(buffer, &input->participant_id);
    return ret;
}

bool uxr_deserialize_OBJK_REPLIER_Representation(
        ucdrBuffer* buffer,
        OBJK_REPLIER_Representation* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_Representation3_Base(buffer, &output->base);
    ret &= uxr_deserialize_ObjectId(buffer, &output->participant_id);
    return ret;
}

bool uxr_serialize_OBJK_DomainParticipant_Binary(
        ucdrBuffer* buffer,
        const OBJK_DomainParticipant_Binary* input)
{
    bool ret = true;
    ret &= ucdr_serialize_bool(buffer, input->optional_domain_reference);
    if (input->optional_domain_reference == true)
    {
        ret &= ucdr_serialize_string(buffer, input->domain_reference);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_qos_profile_reference);
    if (input->optional_qos_profile_reference == true)
    {
        ret &= ucdr_serialize_string(buffer, input->qos_profile_reference);
    }

    return ret;
}

bool uxr_deserialize_OBJK_DomainParticipant_Binary(
        ucdrBuffer* buffer,
        OBJK_DomainParticipant_Binary* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_bool(buffer, &output->optional_domain_reference);
    if (output->optional_domain_reference == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->domain_reference, UXR_STRING_SIZE_MAX);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_qos_profile_reference);
    if (output->optional_qos_profile_reference == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->qos_profile_reference, UXR_STRING_SIZE_MAX);
    }

    return ret;
}

bool uxr_serialize_OBJK_Topic_Binary(
        ucdrBuffer* buffer,
        const OBJK_Topic_Binary* input)
{
    bool ret = true;
    ret &= ucdr_serialize_string(buffer, input->topic_name);
    ret &= ucdr_serialize_bool(buffer, input->optional_type_reference);
    if (input->optional_type_reference == true)
    {
        ret &= ucdr_serialize_string(buffer, input->type_reference);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_type_name);
    if (input->optional_type_name == true)
    {
        ret &= ucdr_serialize_string(buffer, input->type_name);
    }

    return ret;
}

bool uxr_deserialize_OBJK_Topic_Binary(
        ucdrBuffer* buffer,
        OBJK_Topic_Binary* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_string(buffer, output->topic_name, UXR_STRING_SIZE_MAX);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_type_reference);
    if (output->optional_type_reference == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->type_reference, UXR_STRING_SIZE_MAX);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_type_name);
    if (output->optional_type_name == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->type_name, UXR_STRING_SIZE_MAX);
    }

    return ret;
}

bool uxr_serialize_OBJK_Publisher_Binary_Qos(
        ucdrBuffer* buffer,
        const OBJK_Publisher_Binary_Qos* input)
{
    bool ret = true;
    ret &= ucdr_serialize_bool(buffer, input->optional_partitions);
    if (input->optional_partitions == true)
    {
        ret &= uxr_serialize_StringSequence_t(buffer, &input->partitions);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_group_data);
    if (input->optional_group_data == true)
    {
        ret &= uxr_serialize_BinarySequence_t(buffer, &input->group_data);
    }

    return ret;
}

bool uxr_deserialize_OBJK_Publisher_Binary_Qos(
        ucdrBuffer* buffer,
        OBJK_Publisher_Binary_Qos* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_bool(buffer, &output->optional_partitions);
    if (output->optional_partitions == true)
    {
        ret &= uxr_deserialize_StringSequence_t(buffer, &output->partitions);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_group_data);
    if (output->optional_group_data == true)
    {
        ret &= uxr_deserialize_BinarySequence_t(buffer, &output->group_data);
    }

    return ret;
}

bool uxr_serialize_OBJK_Publisher_Binary(
        ucdrBuffer* buffer,
        const OBJK_Publisher_Binary* input)
{
    bool ret = true;
    ret &= ucdr_serialize_bool(buffer, input->optional_publisher_name);
    if (input->optional_publisher_name == true)
    {
        ret &= ucdr_serialize_string(buffer, input->publisher_name);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_qos);
    if (input->optional_qos == true)
    {
        ret &= uxr_serialize_OBJK_Publisher_Binary_Qos(buffer, &input->qos);
    }

    return ret;
}

bool uxr_deserialize_OBJK_Publisher_Binary(
        ucdrBuffer* buffer,
        OBJK_Publisher_Binary* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_bool(buffer, &output->optional_publisher_name);
    if (output->optional_publisher_name == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->publisher_name, UXR_STRING_SIZE_MAX);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_qos);
    if (output->optional_qos == true)
    {
        ret &= uxr_deserialize_OBJK_Publisher_Binary_Qos(buffer, &output->qos);
    }

    return ret;
}

bool uxr_serialize_OBJK_Subscriber_Binary_Qos(
        ucdrBuffer* buffer,
        const OBJK_Subscriber_Binary_Qos* input)
{
    bool ret = true;
    ret &= ucdr_serialize_bool(buffer, input->optional_partitions);
    if (input->optional_partitions == true)
    {
        ret &= uxr_serialize_StringSequence_t(buffer, &input->partitions);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_group_data);
    if (input->optional_group_data == true)
    {
        ret &= uxr_serialize_BinarySequence_t(buffer, &input->group_data);
    }

    return ret;
}

bool uxr_deserialize_OBJK_Subscriber_Binary_Qos(
        ucdrBuffer* buffer,
        OBJK_Subscriber_Binary_Qos* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_bool(buffer, &output->optional_partitions);
    if (output->optional_partitions == true)
    {
        ret &= uxr_deserialize_StringSequence_t(buffer, &output->partitions);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_group_data);
    if (output->optional_group_data == true)
    {
        ret &= uxr_deserialize_BinarySequence_t(buffer, &output->group_data);
    }

    return ret;
}

bool uxr_serialize_OBJK_Subscriber_Binary(
        ucdrBuffer* buffer,
        const OBJK_Subscriber_Binary* input)
{
    bool ret = true;
    ret &= ucdr_serialize_bool(buffer, input->optional_subscriber_name);
    if (input->optional_subscriber_name == true)
    {
        ret &= ucdr_serialize_string(buffer, input->subscriber_name);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_qos);
    if (input->optional_qos == true)
    {
        ret &= uxr_serialize_OBJK_Subscriber_Binary_Qos(buffer, &input->qos);
    }

    return ret;
}

bool uxr_deserialize_OBJK_Subscriber_Binary(
        ucdrBuffer* buffer,
        OBJK_Subscriber_Binary* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_bool(buffer, &output->optional_subscriber_name);
    if (output->optional_subscriber_name == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->subscriber_name, UXR_STRING_SIZE_MAX);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_qos);
    if (output->optional_qos == true)
    {
        ret &= uxr_deserialize_OBJK_Subscriber_Binary_Qos(buffer, &output->qos);
    }

    return ret;
}

bool uxr_serialize_OBJK_Endpoint_QosBinary(
        ucdrBuffer* buffer,
        const OBJK_Endpoint_QosBinary* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint16_t(buffer, input->qos_flags);
    ret &= ucdr_serialize_bool(buffer, input->optional_history_depth);
    if (input->optional_history_depth == true)
    {
        ret &= ucdr_serialize_uint16_t(buffer, input->history_depth);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_deadline_msec);
    if (input->optional_deadline_msec == true)
    {
        ret &= ucdr_serialize_uint32_t(buffer, input->deadline_msec);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_lifespan_msec);
    if (input->optional_lifespan_msec == true)
    {
        ret &= ucdr_serialize_uint32_t(buffer, input->lifespan_msec);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_user_data);
    if (input->optional_user_data == true)
    {
        ret &= uxr_serialize_BinarySequence_t(buffer, (BinarySequence_t*) &input->user_data);
    }

    return ret;
}

bool uxr_deserialize_OBJK_Endpoint_QosBinary(
        ucdrBuffer* buffer,
        OBJK_Endpoint_QosBinary* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint16_t(buffer, &output->qos_flags);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_history_depth);
    if (output->optional_history_depth == true)
    {
        ret &= ucdr_deserialize_uint16_t(buffer, &output->history_depth);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_deadline_msec);
    if (output->optional_deadline_msec == true)
    {
        ret &= ucdr_deserialize_uint32_t(buffer, &output->deadline_msec);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_lifespan_msec);
    if (output->optional_lifespan_msec == true)
    {
        ret &= ucdr_deserialize_uint32_t(buffer, &output->lifespan_msec);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_user_data);
    if (output->optional_user_data == true)
    {
        ret &= uxr_deserialize_BinarySequence_t(buffer, (BinarySequence_t*) &output->user_data);
    }

    return ret;
}

bool uxr_serialize_OBJK_DataWriter_Binary_Qos(
        ucdrBuffer* buffer,
        const OBJK_DataWriter_Binary_Qos* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_Endpoint_QosBinary(buffer, &input->base);
    ret &= ucdr_serialize_bool(buffer, input->optional_ownership_strength);
    if (input->optional_ownership_strength == true)
    {
        ret &= ucdr_serialize_uint64_t(buffer, input->ownership_strength);
    }

    return ret;
}

bool uxr_deserialize_OBJK_DataWriter_Binary_Qos(
        ucdrBuffer* buffer,
        OBJK_DataWriter_Binary_Qos* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_Endpoint_QosBinary(buffer, &output->base);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_ownership_strength);
    if (output->optional_ownership_strength == true)
    {
        ret &= ucdr_deserialize_uint64_t(buffer, &output->ownership_strength);
    }

    return ret;
}

bool uxr_serialize_OBJK_DataReader_Binary_Qos(
        ucdrBuffer* buffer,
        const OBJK_DataReader_Binary_Qos* input)
{
    bool ret = true;
    ret &= uxr_serialize_OBJK_Endpoint_QosBinary(buffer, &input->base);
    ret &= ucdr_serialize_bool(buffer, input->optional_timebasedfilter_msec);
    if (input->optional_timebasedfilter_msec == true)
    {
        ret &= ucdr_serialize_uint64_t(buffer, input->timebasedfilter_msec);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_contentbased_filter);
    if (input->optional_contentbased_filter == true)
    {
        ret &= ucdr_serialize_string(buffer, input->contentbased_filter);
    }

    return ret;
}

bool uxr_deserialize_OBJK_DataReader_Binary_Qos(
        ucdrBuffer* buffer,
        OBJK_DataReader_Binary_Qos* output)
{
    bool ret = true;
    ret &= uxr_deserialize_OBJK_Endpoint_QosBinary(buffer, &output->base);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_timebasedfilter_msec);
    if (output->optional_timebasedfilter_msec == true)
    {
        ret &= ucdr_deserialize_uint64_t(buffer, &output->timebasedfilter_msec);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_contentbased_filter);
    if (output->optional_contentbased_filter == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->contentbased_filter, UXR_STRING_SIZE_MAX);
    }

    return ret;
}

bool uxr_serialize_OBJK_DataReader_Binary(
        ucdrBuffer* buffer,
        const OBJK_DataReader_Binary* input)
{
    bool ret = true;
    ret &= uxr_serialize_ObjectId(buffer, &input->topic_id);
    ret &= ucdr_serialize_bool(buffer, input->optional_qos);
    if (input->optional_qos == true)
    {
        ret &= uxr_serialize_OBJK_DataReader_Binary_Qos(buffer, &input->qos);
    }

    return ret;
}

bool uxr_deserialize_OBJK_DataReader_Binary(
        ucdrBuffer* buffer,
        OBJK_DataReader_Binary* output)
{
    bool ret = true;
    ret &= uxr_deserialize_ObjectId(buffer, &output->topic_id);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_qos);
    if (output->optional_qos == true)
    {
        ret &= uxr_deserialize_OBJK_DataReader_Binary_Qos(buffer, &output->qos);
    }

    return ret;
}

bool uxr_serialize_OBJK_DataWriter_Binary(
        ucdrBuffer* buffer,
        const OBJK_DataWriter_Binary* input)
{
    bool ret = true;
    ret &= uxr_serialize_ObjectId(buffer, &input->topic_id);
    ret &= ucdr_serialize_bool(buffer, input->optional_qos);
    if (input->optional_qos == true)
    {
        ret &= uxr_serialize_OBJK_DataWriter_Binary_Qos(buffer, &input->qos);
    }

    return ret;
}

bool uxr_deserialize_OBJK_DataWriter_Binary(
        ucdrBuffer* buffer,
        OBJK_DataWriter_Binary* output)
{
    bool ret = true;
    ret &= uxr_deserialize_ObjectId(buffer, &output->topic_id);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_qos);
    if (output->optional_qos == true)
    {
        ret &= uxr_deserialize_OBJK_DataWriter_Binary_Qos(buffer, &output->qos);
    }

    return ret;
}

bool uxr_serialize_OBJK_Requester_Binary(
        ucdrBuffer* buffer,
        const OBJK_Requester_Binary* input)
{
    bool ret = true;
    ret &= ucdr_serialize_string(buffer, input->service_name);
    ret &= ucdr_serialize_string(buffer, input->request_type);
    ret &= ucdr_serialize_string(buffer, input->reply_type);
    ret &= ucdr_serialize_bool(buffer, input->optional_request_topic_name);
    if (input->optional_request_topic_name == true)
    {
        ret &= ucdr_serialize_string(buffer, input->request_topic_name);
    }
    ret &= ucdr_serialize_bool(buffer, input->optional_reply_topic_name);
    if (input->optional_reply_topic_name == true)
    {
        ret &= ucdr_serialize_string(buffer, input->reply_topic_name);
    }

    return ret;
}

bool uxr_deserialize_OBJK_Requester_Binary(
        ucdrBuffer* buffer,
        OBJK_Requester_Binary* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_string(buffer, output->service_name, UXR_STRING_SIZE_MAX);
    ret &= ucdr_deserialize_string(buffer, output->request_type, UXR_STRING_SIZE_MAX);
    ret &= ucdr_deserialize_string(buffer, output->reply_type, UXR_STRING_SIZE_MAX);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_request_topic_name);
    if (output->optional_request_topic_name == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->request_topic_name, UXR_STRING_SIZE_MAX);
    }
    ret &= ucdr_deserialize_bool(buffer, &output->optional_reply_topic_name);
    if (output->optional_reply_topic_name == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->reply_topic_name, UXR_STRING_SIZE_MAX);
    }

    return ret;
}

bool uxr_serialize_OBJK_Replier_Binary(
        ucdrBuffer* buffer,
        const OBJK_Replier_Binary* input)
{
    bool ret = true;
    ret &= ucdr_serialize_string(buffer, input->service_name);
    ret &= ucdr_serialize_string(buffer, input->request_type);
    ret &= ucdr_serialize_string(buffer, input->reply_type);
    ret &= ucdr_serialize_bool(buffer, input->optional_request_topic_name);
    if (input->optional_request_topic_name == true)
    {
        ret &= ucdr_serialize_string(buffer, input->request_topic_name);
    }
    ret &= ucdr_serialize_bool(buffer, input->optional_reply_topic_name);
    if (input->optional_reply_topic_name == true)
    {
        ret &= ucdr_serialize_string(buffer, input->reply_topic_name);
    }

    return ret;
}

bool uxr_deserialize_OBJK_Replier_Binary(
        ucdrBuffer* buffer,
        OBJK_Replier_Binary* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_string(buffer, output->service_name, UXR_STRING_SIZE_MAX);
    ret &= ucdr_deserialize_string(buffer, output->request_type, UXR_STRING_SIZE_MAX);
    ret &= ucdr_deserialize_string(buffer, output->reply_type, UXR_STRING_SIZE_MAX);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_request_topic_name);
    if (output->optional_request_topic_name == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->request_topic_name, UXR_STRING_SIZE_MAX);
    }
    ret &= ucdr_deserialize_bool(buffer, &output->optional_reply_topic_name);
    if (output->optional_reply_topic_name == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->reply_topic_name, UXR_STRING_SIZE_MAX);
    }

    return ret;
}

bool uxr_serialize_ObjectVariant(
        ucdrBuffer* buffer,
        const ObjectVariant* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->kind);
    if (ret)
    {
        switch (input->kind)
        {
            case DDS_XRCE_OBJK_AGENT:
                ret &= uxr_serialize_AGENT_Representation(buffer, &input->_.agent);
                break;
            case DDS_XRCE_OBJK_CLIENT:
                ret &= uxr_serialize_CLIENT_Representation(buffer, &input->_.client);
                break;
            case DDS_XRCE_OBJK_APPLICATION:
                ret &= uxr_serialize_OBJK_APPLICATION_Representation(buffer, &input->_.application);
                break;
            case DDS_XRCE_OBJK_PARTICIPANT:
                ret &= uxr_serialize_OBJK_PARTICIPANT_Representation(buffer, &input->_.participant);
                break;
            case DDS_XRCE_OBJK_QOSPROFILE:
                ret &= uxr_serialize_OBJK_QOSPROFILE_Representation(buffer, &input->_.qos_profile);
                break;
            case DDS_XRCE_OBJK_TYPE:
                ret &= uxr_serialize_OBJK_TYPE_Representation(buffer, &input->_.type);
                break;
            case DDS_XRCE_OBJK_TOPIC:
                ret &= uxr_serialize_OBJK_TOPIC_Representation(buffer, &input->_.topic);
                break;
            case DDS_XRCE_OBJK_PUBLISHER:
                ret &= uxr_serialize_OBJK_PUBLISHER_Representation(buffer, &input->_.publisher);
                break;
            case DDS_XRCE_OBJK_SUBSCRIBER:
                ret &= uxr_serialize_OBJK_SUBSCRIBER_Representation(buffer, &input->_.subscriber);
                break;
            case DDS_XRCE_OBJK_DATAWRITER:
                ret &= uxr_serialize_DATAWRITER_Representation(buffer, &input->_.data_writer);
                break;
            case DDS_XRCE_OBJK_DATAREADER:
                ret &= uxr_serialize_DATAREADER_Representation(buffer, &input->_.data_reader);
                break;
            case DDS_XRCE_OBJK_REQUESTER:
                ret &= uxr_serialize_OBJK_REQUESTER_Representation(buffer, &input->_.requester);
                break;
            case DDS_XRCE_OBJK_REPLIER:
                ret &= uxr_serialize_OBJK_REPLIER_Representation(buffer, &input->_.replier);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_deserialize_ObjectVariant(
        ucdrBuffer* buffer,
        ObjectVariant* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->kind);
    if (ret)
    {
        switch (output->kind)
        {
            case DDS_XRCE_OBJK_AGENT:
                ret &= uxr_deserialize_AGENT_Representation(buffer, &output->_.agent);
                break;
            case DDS_XRCE_OBJK_CLIENT:
                ret &= uxr_deserialize_CLIENT_Representation(buffer, &output->_.client);
                break;
            case DDS_XRCE_OBJK_APPLICATION:
                ret &= uxr_deserialize_OBJK_APPLICATION_Representation(buffer, &output->_.application);
                break;
            case DDS_XRCE_OBJK_PARTICIPANT:
                ret &= uxr_deserialize_OBJK_PARTICIPANT_Representation(buffer, &output->_.participant);
                break;
            case DDS_XRCE_OBJK_QOSPROFILE:
                ret &= uxr_deserialize_OBJK_QOSPROFILE_Representation(buffer, &output->_.qos_profile);
                break;
            case DDS_XRCE_OBJK_TYPE:
                ret &= uxr_deserialize_OBJK_TYPE_Representation(buffer, &output->_.type);
                break;
            case DDS_XRCE_OBJK_TOPIC:
                ret &= uxr_deserialize_OBJK_TOPIC_Representation(buffer, &output->_.topic);
                break;
            case DDS_XRCE_OBJK_PUBLISHER:
                ret &= uxr_deserialize_OBJK_PUBLISHER_Representation(buffer, &output->_.publisher);
                break;
            case DDS_XRCE_OBJK_SUBSCRIBER:
                ret &= uxr_deserialize_OBJK_SUBSCRIBER_Representation(buffer, &output->_.subscriber);
                break;
            case DDS_XRCE_OBJK_DATAWRITER:
                ret &= uxr_deserialize_DATAWRITER_Representation(buffer, &output->_.data_writer);
                break;
            case DDS_XRCE_OBJK_DATAREADER:
                ret &= uxr_deserialize_DATAREADER_Representation(buffer, &output->_.data_reader);
                break;
            case DDS_XRCE_OBJK_REQUESTER:
                ret &= uxr_deserialize_OBJK_REQUESTER_Representation(buffer, &output->_.requester);
                break;
            case DDS_XRCE_OBJK_REPLIER:
                ret &= uxr_deserialize_OBJK_REPLIER_Representation(buffer, &output->_.replier);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_serialize_CreationMode(
        ucdrBuffer* buffer,
        const CreationMode* input)
{
    bool ret = true;
    ret &= ucdr_serialize_bool(buffer, input->reuse);
    ret &= ucdr_serialize_bool(buffer, input->replace);
    return ret;
}

bool uxr_deserialize_CreationMode(
        ucdrBuffer* buffer,
        CreationMode* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_bool(buffer, &output->reuse);
    ret &= ucdr_deserialize_bool(buffer, &output->replace);
    return ret;
}

bool uxr_serialize_RequestId(
        ucdrBuffer* buffer,
        const RequestId* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->data, 2);
    return ret;
}

bool uxr_deserialize_RequestId(
        ucdrBuffer* buffer,
        RequestId* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->data, 2);
    return ret;
}

bool uxr_serialize_ResultStatus(
        ucdrBuffer* buffer,
        const ResultStatus* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->status);
    ret &= ucdr_serialize_uint8_t(buffer, input->implementation_status);
    return ret;
}

bool uxr_deserialize_ResultStatus(
        ucdrBuffer* buffer,
        ResultStatus* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->status);
    ret &= ucdr_deserialize_uint8_t(buffer, &output->implementation_status);
    return ret;
}

bool uxr_serialize_BaseObjectRequest(
        ucdrBuffer* buffer,
        const BaseObjectRequest* input)
{
    bool ret = true;
    ret &= uxr_serialize_RequestId(buffer, &input->request_id);
    ret &= uxr_serialize_ObjectId(buffer, &input->object_id);
    return ret;
}

bool uxr_deserialize_BaseObjectRequest(
        ucdrBuffer* buffer,
        BaseObjectRequest* output)
{
    bool ret = true;
    ret &= uxr_deserialize_RequestId(buffer, &output->request_id);
    ret &= uxr_deserialize_ObjectId(buffer, &output->object_id);
    return ret;
}

bool uxr_serialize_AGENT_ActivityInfo(
        ucdrBuffer* buffer,
        const AGENT_ActivityInfo* input)
{
    bool ret = true;
    ret &= ucdr_serialize_int16_t(buffer, input->availability);
    ret &= uxr_serialize_TransportLocatorSeq(buffer, &input->address_seq);
    return ret;
}

bool uxr_deserialize_AGENT_ActivityInfo(
        ucdrBuffer* buffer,
        AGENT_ActivityInfo* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_int16_t(buffer, &output->availability);
    ret &= uxr_deserialize_TransportLocatorSeq(buffer, &output->address_seq);
    return ret;
}

bool uxr_serialize_DATAREADER_ActivityInfo(
        ucdrBuffer* buffer,
        const DATAREADER_ActivityInfo* input)
{
    bool ret = true;
    ret &= ucdr_serialize_int16_t(buffer, input->highest_acked_num);
    return ret;
}

bool uxr_deserialize_DATAREADER_ActivityInfo(
        ucdrBuffer* buffer,
        DATAREADER_ActivityInfo* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_int16_t(buffer, &output->highest_acked_num);
    return ret;
}

bool uxr_serialize_DATAWRITER_ActivityInfo(
        ucdrBuffer* buffer,
        const DATAWRITER_ActivityInfo* input)
{
    bool ret = true;
    ret &= ucdr_serialize_int16_t(buffer, input->stream_seq_num);
    ret &= ucdr_serialize_uint64_t(buffer, input->sample_seq_num);
    return ret;
}

bool uxr_deserialize_DATAWRITER_ActivityInfo(
        ucdrBuffer* buffer,
        DATAWRITER_ActivityInfo* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_int16_t(buffer, &output->stream_seq_num);
    ret &= ucdr_deserialize_uint64_t(buffer, &output->sample_seq_num);
    return ret;
}

bool uxr_serialize_ActivityInfoVariant(
        ucdrBuffer* buffer,
        const ActivityInfoVariant* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->kind);
    if (ret)
    {
        switch (input->kind)
        {
            case DDS_XRCE_OBJK_AGENT:
                ret &= uxr_serialize_AGENT_ActivityInfo(buffer, &input->_.agent);
                break;
            case DDS_XRCE_OBJK_DATAWRITER:
                ret &= uxr_serialize_DATAWRITER_ActivityInfo(buffer, &input->_.data_writer);
                break;
            case DDS_XRCE_OBJK_DATAREADER:
                ret &= uxr_serialize_DATAREADER_ActivityInfo(buffer, &input->_.data_reader);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_deserialize_ActivityInfoVariant(
        ucdrBuffer* buffer,
        ActivityInfoVariant* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->kind);
    if (ret)
    {
        switch (output->kind)
        {
            case DDS_XRCE_OBJK_AGENT:
                ret &= uxr_deserialize_AGENT_ActivityInfo(buffer, &output->_.agent);
                break;
            case DDS_XRCE_OBJK_DATAWRITER:
                ret &= uxr_deserialize_DATAWRITER_ActivityInfo(buffer, &output->_.data_writer);
                break;
            case DDS_XRCE_OBJK_DATAREADER:
                ret &= uxr_deserialize_DATAREADER_ActivityInfo(buffer, &output->_.data_reader);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_serialize_ObjectInfo(
        ucdrBuffer* buffer,
        const ObjectInfo* input)
{
    bool ret = true;
    ret &= ucdr_serialize_bool(buffer, input->optional_config);
    if (input->optional_config == true)
    {
        ret &= uxr_serialize_ObjectVariant(buffer, &input->config);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_activity);
    if (input->optional_activity == true)
    {
        ret &= uxr_serialize_ActivityInfoVariant(buffer, &input->activity);
    }

    return ret;
}

bool uxr_deserialize_ObjectInfo(
        ucdrBuffer* buffer,
        ObjectInfo* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_bool(buffer, &output->optional_config);
    if (output->optional_config == true)
    {
        ret &= uxr_deserialize_ObjectVariant(buffer, &output->config);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_activity);
    if (output->optional_activity == true)
    {
        ret &= uxr_deserialize_ActivityInfoVariant(buffer, &output->activity);
    }

    return ret;
}

bool uxr_serialize_BaseObjectReply(
        ucdrBuffer* buffer,
        const BaseObjectReply* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->related_request);
    ret &= uxr_serialize_ResultStatus(buffer, &input->result);
    return ret;
}

bool uxr_deserialize_BaseObjectReply(
        ucdrBuffer* buffer,
        BaseObjectReply* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->related_request);
    ret &= uxr_deserialize_ResultStatus(buffer, &output->result);
    return ret;
}

bool uxr_serialize_DataDeliveryControl(
        ucdrBuffer* buffer,
        const DataDeliveryControl* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint16_t(buffer, input->max_samples);
    ret &= ucdr_serialize_uint16_t(buffer, input->max_elapsed_time);
    ret &= ucdr_serialize_uint16_t(buffer, input->max_bytes_per_seconds);
    ret &= ucdr_serialize_uint16_t(buffer, input->min_pace_period);
    return ret;
}

bool uxr_deserialize_DataDeliveryControl(
        ucdrBuffer* buffer,
        DataDeliveryControl* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint16_t(buffer, &output->max_samples);
    ret &= ucdr_deserialize_uint16_t(buffer, &output->max_elapsed_time);
    ret &= ucdr_deserialize_uint16_t(buffer, &output->max_bytes_per_seconds);
    ret &= ucdr_deserialize_uint16_t(buffer, &output->min_pace_period);
    return ret;
}

bool uxr_serialize_ReadSpecification(
        ucdrBuffer* buffer,
        const ReadSpecification* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->preferred_stream_id);
    ret &= ucdr_serialize_uint8_t(buffer, input->data_format);
    ret &= ucdr_serialize_bool(buffer, input->optional_content_filter_expression);
    if (input->optional_content_filter_expression == true)
    {
        ret &= ucdr_serialize_string(buffer, input->content_filter_expression);
    }

    ret &= ucdr_serialize_bool(buffer, input->optional_delivery_control);
    if (input->optional_delivery_control == true)
    {
        ret &= uxr_serialize_DataDeliveryControl(buffer, &input->delivery_control);
    }

    return ret;
}

bool uxr_deserialize_ReadSpecification(
        ucdrBuffer* buffer,
        ReadSpecification* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->preferred_stream_id);
    ret &= ucdr_deserialize_uint8_t(buffer, &output->data_format);
    ret &= ucdr_deserialize_bool(buffer, &output->optional_content_filter_expression);
    if (output->optional_content_filter_expression == true)
    {
        ret &= ucdr_deserialize_string(buffer, output->content_filter_expression, UXR_STRING_SIZE_MAX);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->optional_delivery_control);
    if (output->optional_delivery_control == true)
    {
        ret &= uxr_deserialize_DataDeliveryControl(buffer, &output->delivery_control);
    }

    return ret;
}

bool uxr_serialize_SeqNumberAndTimestamp(
        ucdrBuffer* buffer,
        const SeqNumberAndTimestamp* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint32_t(buffer, input->sequence_number);
    ret &= ucdr_serialize_uint32_t(buffer, input->session_time_offset);
    return ret;
}

bool uxr_deserialize_SeqNumberAndTimestamp(
        ucdrBuffer* buffer,
        SeqNumberAndTimestamp* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint32_t(buffer, &output->sequence_number);
    ret &= ucdr_deserialize_uint32_t(buffer, &output->session_time_offset);
    return ret;
}

bool uxr_serialize_SampleInfoDetail(
        ucdrBuffer* buffer,
        const SampleInfoDetail* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint32_t(buffer, input->format);
    if (ret)
    {
        switch (input->format)
        {
            case FORMAT_EMPTY:
                ret &= ucdr_serialize_uint32_t(buffer, input->_.sequence_number);
                break;
            case FORMAT_SEQNUM:
                ret &= ucdr_serialize_uint32_t(buffer, input->_.session_time_offset);
                break;
            case FORMAT_TIMESTAMP:
                ret &= uxr_serialize_SeqNumberAndTimestamp(buffer, &input->_.seqnum_n_timestamp);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_deserialize_SampleInfoDetail(
        ucdrBuffer* buffer,
        SampleInfoDetail* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint32_t(buffer, &output->format);
    if (ret)
    {
        switch (output->format)
        {
            case FORMAT_EMPTY:
                ret &= ucdr_deserialize_uint32_t(buffer, &output->_.sequence_number);
                break;
            case FORMAT_SEQNUM:
                ret &= ucdr_deserialize_uint32_t(buffer, &output->_.session_time_offset);
                break;
            case FORMAT_TIMESTAMP:
                ret &= uxr_deserialize_SeqNumberAndTimestamp(buffer, &output->_.seqnum_n_timestamp);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_serialize_SampleInfo(
        ucdrBuffer* buffer,
        const SampleInfo* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->state);
    ret &= uxr_serialize_SampleInfoDetail(buffer, &input->detail);
    return ret;
}

bool uxr_deserialize_SampleInfo(
        ucdrBuffer* buffer,
        SampleInfo* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->state);
    ret &= uxr_deserialize_SampleInfoDetail(buffer, &output->detail);
    return ret;
}

bool uxr_serialize_SampleInfoDelta(
        ucdrBuffer* buffer,
        const SampleInfoDelta* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->state);
    ret &= ucdr_serialize_uint8_t(buffer, input->seq_number_delta);
    ret &= ucdr_serialize_uint16_t(buffer, input->timestamp_delta);
    return ret;
}

bool uxr_deserialize_SampleInfoDelta(
        ucdrBuffer* buffer,
        SampleInfoDelta* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->state);
    ret &= ucdr_deserialize_uint8_t(buffer, &output->seq_number_delta);
    ret &= ucdr_deserialize_uint16_t(buffer, &output->timestamp_delta);
    return ret;
}

bool uxr_serialize_SampleData(
        ucdrBuffer* buffer,
        const SampleData* input)
{
    return ucdr_serialize_sequence_uint8_t(buffer, input->data, input->size);
}

bool uxr_deserialize_SampleData(
        ucdrBuffer* buffer,
        SampleData* output)
{
    return ucdr_deserialize_sequence_uint8_t(buffer, output->data, UXR_SAMPLE_DATA_SIZE_MAX, &output->size);
}

bool uxr_serialize_SampleDataSeq(
        ucdrBuffer* buffer,
        const SampleDataSeq* input)
{
    bool ret = ucdr_serialize_uint32_t(buffer, input->size);
    for (uint32_t i = 0; i < input->size && ret; i++)
    {
        ret = uxr_serialize_SampleData(buffer, &input->data[i]);
    }
    return ret;
}

bool uxr_deserialize_SampleDataSeq(
        ucdrBuffer* buffer,
        SampleDataSeq* output)
{
    bool ret = ucdr_deserialize_uint32_t(buffer, &output->size);
    if (output->size > UXR_SAMPLE_DATA_SEQUENCE_MAX)
    {
        buffer->error = true;
        ret = false;
    }
    else
    {
        for (uint32_t i = 0; i < output->size && ret; i++)
        {
            ret = uxr_deserialize_SampleData(buffer, &output->data[i]);
        }
    }
    return ret;
}

bool uxr_serialize_Sample(
        ucdrBuffer* buffer,
        const Sample* input)
{
    bool ret = true;
    ret &= uxr_serialize_SampleInfo(buffer, &input->info);
    ret &= uxr_serialize_SampleData(buffer, &input->data);
    return ret;
}

bool uxr_deserialize_Sample(
        ucdrBuffer* buffer,
        Sample* output)
{
    bool ret = true;
    ret &= uxr_deserialize_SampleInfo(buffer, &output->info);
    ret &= uxr_deserialize_SampleData(buffer, &output->data);
    return ret;
}

bool uxr_serialize_SampleSeq(
        ucdrBuffer* buffer,
        const SampleSeq* input)
{
    bool ret = ucdr_serialize_uint32_t(buffer, input->size);
    for (uint32_t i = 0; i < input->size && ret; i++)
    {
        ret = uxr_serialize_Sample(buffer, &input->data[i]);
    }
    return ret;
}

bool uxr_deserialize_SampleSeq(
        ucdrBuffer* buffer,
        SampleSeq* output)
{
    bool ret = ucdr_deserialize_uint32_t(buffer, &output->size);
    if (output->size > UXR_SAMPLE_SEQUENCE_MAX)
    {
        buffer->error = true;
        ret = false;
    }
    else
    {
        for (uint32_t i = 0; i < output->size && ret; i++)
        {
            ret = uxr_deserialize_Sample(buffer, &output->data[i]);
        }
    }
    return ret;
}

bool uxr_serialize_SampleDelta(
        ucdrBuffer* buffer,
        const SampleDelta* input)
{
    bool ret = true;
    ret &= uxr_serialize_SampleInfoDelta(buffer, &input->info_delta);
    ret &= uxr_serialize_SampleData(buffer, &input->data);
    return ret;
}

bool uxr_deserialize_SampleDelta(
        ucdrBuffer* buffer,
        SampleDelta* output)
{
    bool ret = true;
    ret &= uxr_deserialize_SampleInfoDelta(buffer, &output->info_delta);
    ret &= uxr_deserialize_SampleData(buffer, &output->data);
    return ret;
}

bool uxr_serialize_SampleDeltaSequence(
        ucdrBuffer* buffer,
        const SampleDeltaSequence* input)
{
    bool ret = ucdr_serialize_uint32_t(buffer, input->size);
    for (uint32_t i = 0; i < input->size && ret; i++)
    {
        ret = uxr_serialize_SampleDelta(buffer, &input->data[i]);
    }
    return ret;
}

bool uxr_deserialize_SampleDeltaSequence(
        ucdrBuffer* buffer,
        SampleDeltaSequence* output)
{
    bool ret = ucdr_deserialize_uint32_t(buffer, &output->size);
    if (output->size > UXR_SAMPLE_DELTA_SEQUENCE_MAX)
    {
        buffer->error = true;
        ret = false;
    }
    else
    {
        for (uint32_t i = 0; i < output->size && ret; i++)
        {
            ret = uxr_deserialize_SampleDelta(buffer, &output->data[i]);
        }
    }
    return ret;
}

bool uxr_serialize_PackedSamples(
        ucdrBuffer* buffer,
        const PackedSamples* input)
{
    bool ret = true;
    ret &= uxr_serialize_SampleInfo(buffer, &input->info_base);
    ret &= uxr_serialize_SampleDeltaSequence(buffer, &input->sample_delta_seq);
    return ret;
}

bool uxr_deserialize_PackedSamples(
        ucdrBuffer* buffer,
        PackedSamples* output)
{
    bool ret = true;
    ret &= uxr_deserialize_SampleInfo(buffer, &output->info_base);
    ret &= uxr_deserialize_SampleDeltaSequence(buffer, &output->sample_delta_seq);
    return ret;
}

bool uxr_serialize_SamplePackedSeq(
        ucdrBuffer* buffer,
        const SamplePackedSeq* input)
{
    bool ret = ucdr_serialize_uint32_t(buffer, input->size);
    for (uint32_t i = 0; i < input->size && ret; i++)
    {
        ret = uxr_serialize_PackedSamples(buffer, &input->data[i]);
    }
    return ret;
}

bool uxr_deserialize_SamplePackedSeq(
        ucdrBuffer* buffer,
        SamplePackedSeq* output)
{
    bool ret = ucdr_deserialize_uint32_t(buffer, &output->size);
    if (output->size > UXR_PACKED_SAMPLES_SEQUENCE_MAX)
    {
        buffer->error = true;
        ret = false;
    }
    else
    {
        for (uint32_t i = 0; i < output->size && ret; i++)
        {
            ret = uxr_deserialize_PackedSamples(buffer, &output->data[i]);
        }
    }
    return ret;
}

bool uxr_serialize_DataRepresentation(
        ucdrBuffer* buffer,
        const DataRepresentation* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint8_t(buffer, input->format);
    if (ret)
    {
        switch (input->format)
        {
            case FORMAT_DATA:
                ret &= uxr_serialize_SampleData(buffer, &input->_.data);
                break;
            case FORMAT_SAMPLE:
                ret &= uxr_serialize_Sample(buffer, &input->_.sample);
                break;
            case FORMAT_DATA_SEQ:
                ret &= uxr_serialize_SampleDataSeq(buffer, &input->_.data_seq);
                break;
            case FORMAT_SAMPLE_SEQ:
                ret &= uxr_serialize_SampleSeq(buffer, &input->_.sample_seq);
                break;
            case FORMAT_PACKED_SAMPLES:
                ret &= uxr_serialize_PackedSamples(buffer, &input->_.packed_samples);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_deserialize_DataRepresentation(
        ucdrBuffer* buffer,
        DataRepresentation* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint8_t(buffer, &output->format);
    if (ret)
    {
        switch (output->format)
        {
            case FORMAT_DATA:
                ret &= uxr_deserialize_SampleData(buffer, &output->_.data);
                break;
            case FORMAT_SAMPLE:
                ret &= uxr_deserialize_Sample(buffer, &output->_.sample);
                break;
            case FORMAT_DATA_SEQ:
                ret &= uxr_deserialize_SampleDataSeq(buffer, &output->_.data_seq);
                break;
            case FORMAT_SAMPLE_SEQ:
                ret &= uxr_deserialize_SampleSeq(buffer, &output->_.sample_seq);
                break;
            case FORMAT_PACKED_SAMPLES:
                ret &= uxr_deserialize_PackedSamples(buffer, &output->_.packed_samples);
                break;
            default:
                break;
        }
    }
    return ret;
}

bool uxr_serialize_CREATE_CLIENT_Payload(
        ucdrBuffer* buffer,
        const CREATE_CLIENT_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_CLIENT_Representation(buffer, &input->client_representation);
    return ret;
}

bool uxr_deserialize_CREATE_CLIENT_Payload(
        ucdrBuffer* buffer,
        CREATE_CLIENT_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_CLIENT_Representation(buffer, &output->client_representation);
    return ret;
}

bool uxr_serialize_CREATE_Payload(
        ucdrBuffer* buffer,
        const CREATE_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_ObjectVariant(buffer, &input->object_representation);
    return ret;
}

bool uxr_deserialize_CREATE_Payload(
        ucdrBuffer* buffer,
        CREATE_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_ObjectVariant(buffer, &output->object_representation);
    return ret;
}

bool uxr_serialize_GET_INFO_Payload(
        ucdrBuffer* buffer,
        const GET_INFO_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= ucdr_serialize_uint32_t(buffer, input->info_mask);
    return ret;
}

bool uxr_deserialize_GET_INFO_Payload(
        ucdrBuffer* buffer,
        GET_INFO_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= ucdr_deserialize_uint32_t(buffer, &output->info_mask);
    return ret;
}

bool uxr_serialize_DELETE_Payload(
        ucdrBuffer* buffer,
        const DELETE_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    return ret;
}

bool uxr_deserialize_DELETE_Payload(
        ucdrBuffer* buffer,
        DELETE_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    return ret;
}

bool uxr_serialize_STATUS_AGENT_Payload(
        ucdrBuffer* buffer,
        const STATUS_AGENT_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_ResultStatus(buffer, &input->result);
    ret &= uxr_serialize_AGENT_Representation(buffer, &input->agent_info);
    return ret;
}

bool uxr_deserialize_STATUS_AGENT_Payload(
        ucdrBuffer* buffer,
        STATUS_AGENT_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_ResultStatus(buffer, &output->result);
    ret &= uxr_deserialize_AGENT_Representation(buffer, &output->agent_info);
    return ret;
}

bool uxr_serialize_STATUS_Payload(
        ucdrBuffer* buffer,
        const STATUS_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectReply(buffer, &input->base);
    return ret;
}

bool uxr_deserialize_STATUS_Payload(
        ucdrBuffer* buffer,
        STATUS_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectReply(buffer, &output->base);
    return ret;
}

bool uxr_serialize_INFO_Payload(
        ucdrBuffer* buffer,
        const INFO_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectReply(buffer, &input->base);
    ret &= uxr_serialize_ObjectInfo(buffer, &input->object_info);
    return ret;
}

bool uxr_deserialize_INFO_Payload(
        ucdrBuffer* buffer,
        INFO_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectReply(buffer, &output->base);
    ret &= uxr_deserialize_ObjectInfo(buffer, &output->object_info);
    return ret;
}

bool uxr_serialize_READ_DATA_Payload(
        ucdrBuffer* buffer,
        const READ_DATA_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_ReadSpecification(buffer, &input->read_specification);
    return ret;
}

bool uxr_deserialize_READ_DATA_Payload(
        ucdrBuffer* buffer,
        READ_DATA_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_ReadSpecification(buffer, &output->read_specification);
    return ret;
}

bool uxr_serialize_WRITE_DATA_Payload_Data(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_Data* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    return ret;
}

bool uxr_deserialize_WRITE_DATA_Payload_Data(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_Data* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    return ret;
}

bool uxr_serialize_WRITE_DATA_Payload_Sample(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_Sample* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_Sample(buffer, &input->sample);
    return ret;
}

bool uxr_deserialize_WRITE_DATA_Payload_Sample(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_Sample* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_Sample(buffer, &output->sample);
    return ret;
}

bool uxr_serialize_WRITE_DATA_Payload_DataSeq(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_DataSeq* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_SampleDataSeq(buffer, &input->data_seq);
    return ret;
}

bool uxr_deserialize_WRITE_DATA_Payload_DataSeq(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_DataSeq* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_SampleDataSeq(buffer, &output->data_seq);
    return ret;
}

bool uxr_serialize_WRITE_DATA_Payload_SampleSeq(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_SampleSeq* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_SampleSeq(buffer, &input->sample_seq);
    return ret;
}

bool uxr_deserialize_WRITE_DATA_Payload_SampleSeq(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_SampleSeq* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_SampleSeq(buffer, &output->sample_seq);
    return ret;
}

bool uxr_serialize_WRITE_DATA_Payload_PackedSamples(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_PackedSamples* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_PackedSamples(buffer, &input->packed_samples);
    return ret;
}

bool uxr_deserialize_WRITE_DATA_Payload_PackedSamples(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_PackedSamples* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_PackedSamples(buffer, &output->packed_samples);
    return ret;
}

bool uxr_serialize_DATA_Payload_Data(
        ucdrBuffer* buffer,
        const DATA_Payload_Data* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    return ret;
}

bool uxr_deserialize_DATA_Payload_Data(
        ucdrBuffer* buffer,
        DATA_Payload_Data* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    return ret;
}

bool uxr_serialize_DATA_Payload_Sample(
        ucdrBuffer* buffer,
        const DATA_Payload_Sample* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_Sample(buffer, &input->sample);
    return ret;
}

bool uxr_deserialize_DATA_Payload_Sample(
        ucdrBuffer* buffer,
        DATA_Payload_Sample* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_Sample(buffer, &output->sample);
    return ret;
}

bool uxr_serialize_DATA_Payload_DataSeq(
        ucdrBuffer* buffer,
        const DATA_Payload_DataSeq* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_SampleDataSeq(buffer, &input->data_seq);
    return ret;
}

bool uxr_deserialize_DATA_Payload_DataSeq(
        ucdrBuffer* buffer,
        DATA_Payload_DataSeq* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_SampleDataSeq(buffer, &output->data_seq);
    return ret;
}

bool uxr_serialize_DATA_Payload_SampleSeq(
        ucdrBuffer* buffer,
        const DATA_Payload_SampleSeq* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_SampleSeq(buffer, &input->sample_seq);
    return ret;
}

bool uxr_deserialize_DATA_Payload_SampleSeq(
        ucdrBuffer* buffer,
        DATA_Payload_SampleSeq* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_SampleSeq(buffer, &output->sample_seq);
    return ret;
}

bool uxr_serialize_DATA_Payload_PackedSamples(
        ucdrBuffer* buffer,
        const DATA_Payload_PackedSamples* input)
{
    bool ret = true;
    ret &= uxr_serialize_BaseObjectRequest(buffer, &input->base);
    ret &= uxr_serialize_PackedSamples(buffer, &input->packed_samples);
    return ret;
}

bool uxr_deserialize_DATA_Payload_PackedSamples(
        ucdrBuffer* buffer,
        DATA_Payload_PackedSamples* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectRequest(buffer, &output->base);
    ret &= uxr_deserialize_PackedSamples(buffer, &output->packed_samples);
    return ret;
}

bool uxr_serialize_ACKNACK_Payload(
        ucdrBuffer* buffer,
        const ACKNACK_Payload* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint16_t(buffer, input->first_unacked_seq_num);
    ret &= ucdr_serialize_array_uint8_t(buffer, input->nack_bitmap, 2);
    ret &= ucdr_serialize_uint8_t(buffer, input->stream_id);
    return ret;
}

bool uxr_deserialize_ACKNACK_Payload(
        ucdrBuffer* buffer,
        ACKNACK_Payload* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint16_t(buffer, &output->first_unacked_seq_num);
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->nack_bitmap, 2);
    ret &= ucdr_deserialize_uint8_t(buffer, &output->stream_id);
    return ret;
}

bool uxr_serialize_HEARTBEAT_Payload(
        ucdrBuffer* buffer,
        const HEARTBEAT_Payload* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint16_t(buffer, input->first_unacked_seq_nr);
    ret &= ucdr_serialize_uint16_t(buffer, input->last_unacked_seq_nr);
    ret &= ucdr_serialize_uint8_t(buffer, input->stream_id);
    return ret;
}

bool uxr_deserialize_HEARTBEAT_Payload(
        ucdrBuffer* buffer,
        HEARTBEAT_Payload* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint16_t(buffer, &output->first_unacked_seq_nr);
    ret &= ucdr_deserialize_uint16_t(buffer, &output->last_unacked_seq_nr);
    ret &= ucdr_deserialize_uint8_t(buffer, &output->stream_id);
    return ret;
}

bool uxr_serialize_TIMESTAMP_Payload(
        ucdrBuffer* buffer,
        const TIMESTAMP_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_Time_t(buffer, &input->transmit_timestamp);
    return ret;
}

bool uxr_deserialize_TIMESTAMP_Payload(
        ucdrBuffer* buffer,
        TIMESTAMP_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_Time_t(buffer, &output->transmit_timestamp);
    return ret;
}

bool uxr_serialize_TIMESTAMP_REPLY_Payload(
        ucdrBuffer* buffer,
        const TIMESTAMP_REPLY_Payload* input)
{
    bool ret = true;
    ret &= uxr_serialize_Time_t(buffer, &input->transmit_timestamp);
    ret &= uxr_serialize_Time_t(buffer, &input->receive_timestamp);
    ret &= uxr_serialize_Time_t(buffer, &input->originate_timestamp);
    return ret;
}

bool uxr_deserialize_TIMESTAMP_REPLY_Payload(
        ucdrBuffer* buffer,
        TIMESTAMP_REPLY_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_Time_t(buffer, &output->transmit_timestamp);
    ret &= uxr_deserialize_Time_t(buffer, &output->receive_timestamp);
    ret &= uxr_deserialize_Time_t(buffer, &output->originate_timestamp);
    return ret;
}

bool uxr_serialize_GuidPrefix_t(
        ucdrBuffer* buffer,
        const GuidPrefix_t* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->data, sizeof(GuidPrefix_t));
    return ret;
}

bool uxr_deserialize_GuidPrefix_t(
        ucdrBuffer* buffer,
        GuidPrefix_t* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->data, sizeof(GuidPrefix_t));
    return ret;
}

bool uxr_serialize_EntityId_t(
        ucdrBuffer* buffer,
        const EntityId_t* input)
{
    bool ret = true;
    ret &= ucdr_serialize_array_uint8_t(buffer, input->entityKey, sizeof(((EntityId_t*)0)->entityKey));
    ret &= ucdr_serialize_uint8_t(buffer, input->entityKind);
    return ret;
}

bool uxr_deserialize_EntityId_t(
        ucdrBuffer* buffer,
        EntityId_t* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->entityKey, sizeof(((EntityId_t*)0)->entityKey));
    ret &= ucdr_deserialize_uint8_t(buffer, &output->entityKind);
    return ret;
}

bool uxr_serialize_GUID_t(
        ucdrBuffer* buffer,
        const GUID_t* input)
{
    bool ret = true;
    ret &= uxr_serialize_GuidPrefix_t(buffer, &input->guidPrefix);
    ret &= uxr_serialize_EntityId_t(buffer, &input->entityId);
    return ret;
}

bool uxr_deserialize_GUID_t(
        ucdrBuffer* buffer,
        GUID_t* output)
{
    bool ret = true;
    ret &= uxr_deserialize_GuidPrefix_t(buffer, &output->guidPrefix);
    ret &= uxr_deserialize_EntityId_t(buffer, &output->entityId);
    return ret;
}

bool uxr_serialize_SequenceNumber_t(
        ucdrBuffer* buffer,
        const SequenceNumber_t* input)
{
    bool ret = true;
    ret &= ucdr_serialize_int32_t(buffer, input->high);
    ret &= ucdr_serialize_uint32_t(buffer, input->low);
    return ret;
}

bool uxr_deserialize_SequenceNumber_t(
        ucdrBuffer* buffer,
        SequenceNumber_t* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_int32_t(buffer, &output->high);
    ret &= ucdr_deserialize_uint32_t(buffer, &output->low);
    return ret;
}

bool uxr_serialize_SampleIdentity(
        ucdrBuffer* buffer,
        const SampleIdentity* input)
{
    bool ret = true;
    ret &= uxr_serialize_GUID_t(buffer, &input->writer_guid);
    ret &= uxr_serialize_SequenceNumber_t(buffer, &input->sequence_number);
    return ret;
}

bool uxr_deserialize_SampleIdentity(
        ucdrBuffer* buffer,
        SampleIdentity* output)
{
    bool ret = true;
    ret &= uxr_deserialize_GUID_t(buffer, &output->writer_guid);
    ret &= uxr_deserialize_SequenceNumber_t(buffer, &output->sequence_number);
    return ret;
}

#ifdef PERFORMANCE_TESTING
bool uxr_serialize_PERFORMANCE_Payload(
        ucdrBuffer* buffer,
        const PERFORMANCE_Payload* input)
{
    bool ret = true;
    ret &= ucdr_serialize_uint32_t(buffer, input->epoch_time_lsb);
    ret &= ucdr_serialize_uint32_t(buffer, input->epoch_time_msb);
    ret &= ucdr_serialize_array_uint8_t(buffer, input->buf, input->len);
    return ret;
}

bool uxr_deserialize_PERFORMANCE_Payload(
        ucdrBuffer* buffer,
        PERFORMANCE_Payload* output)
{
    bool ret = true;
    ret &= ucdr_deserialize_uint32_t(buffer, &output->epoch_time_lsb);
    ret &= ucdr_deserialize_uint32_t(buffer, &output->epoch_time_msb);
    ret &= ucdr_deserialize_array_uint8_t(buffer, output->buf, output->len);
    return ret;
}

#endif /* ifdef PERFORMANCE_TESTING */
