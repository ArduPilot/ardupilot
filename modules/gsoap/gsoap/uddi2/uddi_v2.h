/** @mainpage gSOAP UDDI v2

Universal Description, Discovery and Integration, or UDDI, is the name of a
group of web-based registries that expose information about a business or other
entity and its technical interfaces.

See the UDDI v2 specification @url http://uddi.org/pubs/ProgrammersAPI_v2.htm
for more details.

@section UDDI2_CLASSES Classes

See Related Pages for examples.

UDDI Inquire classes

- @ref uddi2__find_USCOREbinding
- @ref uddi2__find_USCOREbusiness
- @ref uddi2__find_USCORErelatedBusinesses
- @ref uddi2__find_USCOREservice
- @ref uddi2__find_USCOREtModel
- @ref uddi2__get_USCOREbindingDetail
- @ref uddi2__get_USCOREbusinessDetail
- @ref uddi2__get_USCOREbusinessDetailExt
- @ref uddi2__get_USCOREserviceDetail
- @ref uddi2__get_USCOREtModelDetail

UDDI Publish classes

- @ref uddi2__add_USCOREpublisherAssertions
- @ref uddi2__delete_USCOREbinding
- @ref uddi2__delete_USCOREbusiness
- @ref uddi2__delete_USCOREservice
- @ref uddi2__delete_USCOREtModel
- @ref uddi2__delete_USCOREpublisherAssertions
- @ref uddi2__discard_USCOREauthToken
- @ref uddi2__get_USCOREassertionStatusReport
- @ref uddi2__get_USCOREauthToken
- @ref uddi2__get_USCOREpublisherAssertions
- @ref uddi2__get_USCOREregisteredInfo
- @ref uddi2__save_USCOREbinding
- @ref uddi2__save_USCOREbusiness
- @ref uddi2__save_USCOREservice
- @ref uddi2__save_USCOREtModel
- @ref uddi2__set_USCOREpublisherAssertions

@section UDDI2_BINDINGS Binding Reference

- @ref InquireSoap
- @ref PublishSoap

*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_binding
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__find_USCOREbinding
@brief

Represents a request to locate bindings that meet the specified requirements.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137712
*/

/**
@fn uddi2__find_USCOREbinding::uddi2__find_USCOREbinding(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the find_USCOREbinding class.
*/

/**
@fn uddi2__find_USCOREbinding::uddi2__find_USCOREbinding(struct soap *soap, const char *tModelKey)
@brief
@param[in] soap gSOAP context
@param[in] tModelKey string

Creates an instance of the find_USCOREbinding class using the specified tModel
key.
*/

/**
@fn uddi2__find_USCOREbinding::uddi2__find_USCOREbinding(struct soap *soap, std::vector<char*> tModelKeys)
@brief
@param[in] soap gSOAP context
@param[in] tModelKeys collection of tModel key strings

Creates an instance of the find_USCOREbinding class using the specified tModel
keys.
*/

/**
@fn uddi2__bindingDetail* uddi2__find_USCOREbinding::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__bindingDetail object or NULL on error

Send a request to a UDDI server to find binding details.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_business
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__find_USCOREbusiness
@brief

Represents a request to locate businesses that meet specific requirements. When
the send method is called, the instance returns a uddi2__businessList object
that contains a list of business that matched the search criteria.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137713

See also @ref example2.
*/

/**
@fn uddi2__find_USCOREbusiness::uddi2__find_USCOREbusiness(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the find_USCOREbusiness class.
*/

/**
@fn uddi2__find_USCOREbusiness::uddi2__find_USCOREbusiness(struct soap *soap, const char *name)
@brief
@param[in] soap gSOAP context

Creates an instance of the find_USCOREbusiness class using the specified name
of the requested business.
*/

/**
@fn uddi2__find_USCOREbusiness::uddi2__find_USCOREbusiness(struct soap *soap, std::vector<uddi2__keyedReference*> keyedReferences)
@brief
@param[in] soap gSOAP context

Creates an instance of the find_USCOREbusiness class using the specified
category references.
*/

/**
@fn uddi2__find_USCOREbusiness::uddi2__find_USCOREbusiness(struct soap *soap, std::vector<char*> tModelKeys)
@brief
@param[in] soap gSOAP context

Creates an instance of the find_USCOREbusiness class using the specified tModel
keys.
*/

/**
@fn uddi2__businessList *uddi2__find_USCOREbusiness::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__businessList object or NULL on error

Send a request to a UDDI server to find a list of businesses.

See also @ref example2.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_relatedBusinesses
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__find_USCORErelatedBusinesses
@brief

Represents a request to locate businesses that are related to a specific
business. When the send method is called, the instance returns a
uddi2__relatedBusinessList object that contains a list of business that matched
the relationship set.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137714
*/

/**
@fn uddi2__find_USCORErelatedBusinesses::uddi2__find_USCORErelatedBusinesses(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the find_USCORErelatedBusiness class.
*/

/**
@fn uddi2__find_USCORErelatedBusinesses::uddi2__find_USCORErelatedBusinesses(struct soap *soap, const char *businessKey)
@brief
@param[in] soap gSOAP context
@param[in] businessKey string

Creates an instance of the find_USCORErelatedBusiness class using the specified
business key.
*/

/**
@fn uddi2__relatedBusinessesList *uddi2__find_USCORErelatedBusinesses::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__relatedBusinessList object or NULL on error

Send a request to a UDDI server to find a list of related businesses.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_service
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__find_USCOREservice
@brief

Represents a request to locate services that meet the specified requirements.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137715

See also @ref example1.
*/

/**
@fn uddi2__find_USCOREservice::uddi2__find_USCOREservice(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the find_USCOREservice class.
*/

/**
@fn uddi2__find_USCOREservice::uddi2__find_USCOREservice(struct soap *soap, const char *name)
@brief
@param[in] soap gSOAP context
@param[in] name of the service

Creates an instance of the find_USCOREservice class using the specified name.
*/

/**
@fn uddi2__find_USCOREservice::uddi2__find_USCOREservice(struct soap *soap, std::vector<uddi2__keyedReference*> keyedReferences)
@brief
@param[in] soap gSOAP context
@param[in] keyedReferences collection of category keys

Creates an instance of the find_USCOREservice class using the specified
category keys.
*/

/**
@fn uddi2__find_USCOREservice::uddi2__find_USCOREservice(struct soap *soap, std::vector<char*> tModelKeys)
@brief
@param[in] soap gSOAP context
@param[in] tModelKeys collection of tModel key strings

Creates an instance of the find_USCOREservice class using the specified tModel
keys.
*/

/**
@fn uddi2__serviceList* uddi2__find_USCOREservice::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__serviceList object or NULL on error

Send a request to a UDDI server to find a list of services.

See also @ref example1.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_tModel
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__find_USCOREtModel
@brief

Represents a request to locate a list of tModel entries that match a set of
specific criteria. The result of a search is a uddi2__tModelList object that
contains information about registered tModel data that matches the criteria.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137716
*/

/**
@fn uddi2__find_USCOREtModel::uddi2__find_USCOREtModel(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the find_USCOREtModel class.
*/

/**
@fn uddi2__find_USCOREtModel::uddi2__find_USCOREtModel(struct soap *soap, const char *name)
@brief
@param[in] soap gSOAP context
@param[in] name of the tModel

Creates an instance of the find_USCOREtModel class using the specified tModel
name.
*/

/**
@fn uddi2__find_USCOREtModel::uddi2__find_USCOREtModel(struct soap *soap, std::vector<uddi2__keyedReference*> keyedReferences)
@brief
@param[in] soap gSOAP context
@param[in] keyedReferences collection of category keys

Creates an instance of the find_USCOREtModel class using the specified category
keys.
*/

/**
@fn uddi2__tModelList* uddi2__find_USCOREtModel::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__tModelList object or NULL on error

Send a request to a UDDI server to find a tModelList.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_bindingDetail
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__get_USCOREbindingDetail
@brief

Represents a request to get binding details from a UDDI server.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137717
*/

/**
@fn uddi2__get_USCOREbindingDetail::uddi2__get_USCOREbindingDetail(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the get_USCOREbindingDetail class.
*/

/**
@fn uddi2__get_USCOREbindingDetail::uddi2__get_USCOREbindingDetail(struct soap *soap, const char *bindingKey)
@brief
@param[in] soap gSOAP context
@param[in] bindingKey string

Creates an instance of the get_USCOREbindingDetail class using the specified
binding key.
*/

/**
@fn uddi2__get_USCOREbindingDetail::uddi2__get_USCOREbindingDetail(struct soap *soap, std::vector<char*> bindingKeys)
@brief
@param[in] soap gSOAP context
@param[in] bindingKeys collection of key strings

Creates an instance of the get_USCOREbindingDetail class using the specified
collection of binding keys.
*/

/**
@fn uddi2__bindingDetail* uddi2__get_USCOREbindingDetail::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__bindingDetail object or NULL on error

Send a request to a UDDI server to get the binding details.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_businessDetail
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__get_USCOREbusinessDetail
@brief

Represents a request to get business details from a UDDI server.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137718 
*/

/**
@fn uddi2__get_USCOREbusinessDetail::uddi2__get_USCOREbusinessDetail(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the get_USCOREbusinessDetail class.
*/

/**
@fn uddi2__get_USCOREbusinessDetail::uddi2__get_USCOREbusinessDetail(struct soap *soap, const char *businessKey)
@brief
@param[in] soap gSOAP context
@param[in] businessKey string

Creates an instance of the get_USCOREbusinessDetail class using the specified
business key.
*/

/**
@fn uddi2__get_USCOREbusinessDetail::uddi2__get_USCOREbusinessDetail(struct soap *soap, std::vector<char*> businessKeys)
@brief
@param[in] soap gSOAP context
@param[in] businessKeys collection of key strings

Creates an instance of the get_USCOREbusinessDetail class using the specified
collection of business keys.
*/

/**
@fn uddi2__businessDetail* uddi2__get_USCOREbusinessDetail::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__businessDetail object or NULL on error

Send a request to a UDDI server to get the business details.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_businessDetailExt
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__get_USCOREbusinessDetailExt
@brief

Represents a request to get business details from a UDDI server.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137719
*/

/**
@fn uddi2__get_USCOREbusinessDetailExt::uddi2__get_USCOREbusinessDetailExt(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the get_USCOREbusinessDetailExt class.
*/

/**
@fn uddi2__get_USCOREbusinessDetailExt::uddi2__get_USCOREbusinessDetailExt(struct soap *soap, const char *businessKey)
@brief
@param[in] soap gSOAP context
@param[in] businessKey string

Creates an instance of the get_USCOREbusinessDetailExt class using the specified
business key.
*/

/**
@fn uddi2__get_USCOREbusinessDetailExt::uddi2__get_USCOREbusinessDetailExt(struct soap *soap, std::vector<char*> businessKeys)
@brief
@param[in] soap gSOAP context
@param[in] businessKeys collection of key strings

Creates an instance of the get_USCOREbusinessDetailExt class using the specified
collection of business keys.
*/

/**
@fn uddi2__businessDetailExt* uddi2__get_USCOREbusinessDetailExt::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__businessDetailExt object or NULL on error

Send a request to a UDDI server to get the business details.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_serviceDetail
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__get_USCOREserviceDetail
@brief

Represents a request to get service details from a UDDI server.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137720
*/

/**
@fn uddi2__get_USCOREserviceDetail::uddi2__get_USCOREserviceDetail(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the get_USCOREserviceDetail class.
*/

/**
@fn uddi2__get_USCOREserviceDetail::uddi2__get_USCOREserviceDetail(struct soap *soap, const char *serviceKey)
@brief
@param[in] soap gSOAP context
@param[in] serviceKey string

Creates an instance of the get_USCOREserviceDetail class using the specified
service key.
*/

/**
@fn uddi2__get_USCOREserviceDetail::uddi2__get_USCOREserviceDetail(struct soap *soap, std::vector<char*> serviceKeys)
@brief
@param[in] soap gSOAP context
@param[in] serviceKeys collection of key strings

Creates an instance of the get_USCOREserviceDetail class using the specified
collection of service keys.
*/

/**
@fn uddi2__serviceDetail* uddi2__get_USCOREserviceDetail::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__serviceDetail object or NULL on error

Send a request to a UDDI server to get the service details.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_tModelDetail
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__get_USCOREtModelDetail
@brief

Represents a request to get tModel details from a UDDI server.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137721
*/

/**
@fn uddi2__get_USCOREtModelDetail::uddi2__get_USCOREtModelDetail(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the get_USCOREtModelDetail class.
*/

/**
@fn uddi2__get_USCOREtModelDetail::uddi2__get_USCOREtModelDetail(struct soap *soap, const char *tModelKey)
@brief
@param[in] soap gSOAP context
@param[in] tModelKey string

Creates an instance of the get_USCOREtModelDetail class using the specified
tModel key.
*/

/**
@fn uddi2__get_USCOREtModelDetail::uddi2__get_USCOREtModelDetail(struct soap *soap, std::vector<char*> tModelKeys)
@brief
@param[in] soap gSOAP context
@param[in] tModelKeys collection of key strings

Creates an instance of the get_USCOREtModelDetail class using the specified
collection of tModel keys.
*/

/**
@fn uddi2__tModelDetail* uddi2__get_USCOREtModelDetail::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__tModelDetail object or NULL on error

Send a request to a UDDI server to get the tModel details.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:add_publisherAssertions
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__add_USCOREpublisherAssertions
@brief

Represents a request to add one or more publisher assertions to the assertion
collection for an individual publisher.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137731
*/

/**
@fn uddi2__add_USCOREpublisherAssertions::uddi2__add_USCOREpublisherAssertions(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the add_USCOREpublisherAssertions class.
*/

/**
@fn uddi2__add_USCOREpublisherAssertions::uddi2__add_USCOREpublisherAssertions(struct soap *soap, std::vector<uddi2__publisherAssertion*> publisherAssertions)
@brief
@param[in] soap gSOAP context
@param[in] publisherAssertions collection of publisher assertions

Creates an instance of the add_USCOREpublisherAssertions class using the
specified publisher assertions.
*/

/**
@fn uddi2__dispositionReport *uddi2__add_USCOREpublisherAssertions::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__dispositionReport object or NULL on error

Send a request to a UDDI server to add one or more publisher assertions to the
assertion collection for an individual publisher.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_binding
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__delete_USCOREbinding
@brief

Represents a request to delete a binding that meets the specified requirements.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137732
*/

/**
@fn uddi2__delete_USCOREbinding::uddi2__delete_USCOREbinding(struct soap *soap)
@brief

Creates an instance of the delete_USCOREbinding class.
*/

/**
@fn uddi2__delete_USCOREbinding::uddi2__delete_USCOREbinding(struct soap *soap, const char *bindingKey)
@brief
@param[in] soap gSOAP context
@param[in] bindingKey a binding key string

Creates an instance of the delete_USCOREbinding class.
*/

/**
@fn uddi2__delete_USCOREbinding::uddi2__delete_USCOREbinding(struct soap *soap, std::vector<char*> bindingKeys)
@brief
@param[in] soap gSOAP context
@param[in] bindingKeys collection of binding keys

Creates an instance of the delete_USCOREbinding class using the specified
binding keys.
*/

/**
@fn uddi2__dispositionReport *uddi2__delete_USCOREbinding::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__dispositionReport object or NULL on error

Send a request to a UDDI service to delete a binding that meets the specified
requirements.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_business
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__delete_USCOREbusiness
@brief

Represents a request to delete a business that meets the specified requirements.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137733
*/

/**
@fn uddi2__delete_USCOREbusiness::uddi2__delete_USCOREbusiness(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the delete_USCOREbusiness class.
*/

/**
@fn uddi2__delete_USCOREbusiness::uddi2__delete_USCOREbusiness(struct soap *soap, const char *businessKey)
@brief
@param[in] soap gSOAP context
@param[in] businessKey a business key string

Creates an instance of the delete_USCOREbusiness class using the specified
business key.
*/

/**
@fn uddi2__delete_USCOREbusiness::uddi2__delete_USCOREbusiness(struct soap *soap, std::vector<char*> businessKeys)
@brief
@param[in] soap gSOAP context
@param[in] businessKeys collection of business keys

Creates an instance of the delete_USCOREbusiness class using the specified
business keys.
*/

/**
@fn uddi2__dispositionReport *uddi2__delete_USCOREbusiness::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__dispositionReport object or NULL on error

Send a request to a UDDI service to delete a business that meets the specified
requirements.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_service
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__delete_USCOREservice
@brief

Represents a request to delete a service that meets the specified requirements.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137735
*/

/**
@fn uddi2__delete_USCOREservice::uddi2__delete_USCOREservice(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the delete_USCOREservice class.
*/

/**
@fn uddi2__delete_USCOREservice::uddi2__delete_USCOREservice(struct soap *soap, const char *serviceKey)
@brief
@param[in] soap gSOAP context
@param[in] serviceKey a service key string

Creates an instance of the delete_USCOREservice class using the specified
service key.
*/

/**
@fn uddi2__delete_USCOREservice::uddi2__delete_USCOREservice(struct soap *soap, std::vector<char*> serviceKeys)
@brief
@param[in] soap gSOAP context
@param[in] serviceKeys collection of service keys

Creates an instance of the delete_USCOREservice class using the specified
service keys.
*/

/**
@fn uddi2__dispositionReport *uddi2__delete_USCOREservice::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__dispositionReport object or NULL on error

Send a request to a UDDI service to delete a service that meets the specified
requirements.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_tModel
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__delete_USCOREtModel
@brief

Represents a request to delete a tModel that meets the specified requirements.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137736
*/

/**
@fn uddi2__delete_USCOREtModel::uddi2__delete_USCOREtModel(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the delete_USCOREtModel class.
*/

/**
@fn uddi2__delete_USCOREtModel::uddi2__delete_USCOREtModel(struct soap *soap, const char *tModelKey)
@brief
@param[in] soap gSOAP context
@param[in] tModelKey a tModel key string

Creates an instance of the delete_USCOREtModel class using the specified
tModel key.
*/

/**
@fn uddi2__delete_USCOREtModel::uddi2__delete_USCOREtModel(struct soap *soap, std::vector<char*> tModelKeys)
@brief
@param[in] soap gSOAP context
@param[in] tModelKeys collection of tModel keys

Creates an instance of the delete_USCOREtModel class using the specified
tModel keys.
*/

/**
@fn uddi2__dispositionReport *uddi2__delete_USCOREtModel::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__dispositionReport object or NULL on error

Send a request to a UDDI service to delete a tModel that meets the specified
requirements.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_publisherAssertions
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__delete_USCOREpublisherAssertions
@brief

Represents a request to delete publisher assertions meeting the specified
requirements.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137734
*/

/**
@fn uddi2__delete_USCOREpublisherAssertions::uddi2__delete_USCOREpublisherAssertions(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the delete_USCOREpublisherAssertions class.
*/

/**
@fn uddi2__delete_USCOREpublisherAssertions::uddi2__delete_USCOREpublisherAssertions(struct soap *soap, std::vector<uddi2__publisherAssertion*> publisherAssertions)
@brief
@param[in] soap gSOAP context
@param[in] publisherAssertions a collection of publisher assertions

Creates an instance of the delete_USCOREpublisherAssertions class using the
specified collection of publisher assertions.
*/

/**
@fn uddi2__dispositionReport *uddi2__delete_USCOREpublisherAssertions::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__dispositionReport object or NULL on error

Send a request to a UDDI service to delete publisher assertions meeting the
specified requirements.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:discard_authToken
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__discard_USCOREauthToken
@brief

Represents a request to discard an authorization token obtained with the
get_USCOREauthToken class.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137737
*/

/**
@fn uddi2__discard_USCOREauthToken::uddi2__discard_USCOREauthToken(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the delete_USCOREauthToken class.
*/

/**
@fn uddi2__discard_USCOREauthToken::uddi2__discard_USCOREauthToken(struct soap *soap, const char *authInfo)
@brief
@param[in] soap gSOAP context
@param[in] authInfo authorization token provided by the UDDI server

Creates an instance of the delete_USCOREauthToken class using the specified
authentication token string.
*/

/**
@fn uddi2__dispositionReport *uddi2__discard_USCOREauthToken::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__dispositionReport object or NULL on error

Send a request to a UDDI service to discard an authentication token.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_assertionStatusReport
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__get_USCOREassertionStatusReport
@brief

Provides the ability to determine the status of current and outstanding
publisher assertions. The results can be restricted by setting the
completionStatus property.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137738
*/

/**
@fn uddi2__get_USCOREassertionStatusReport::uddi2__get_USCOREassertionStatusReport(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the get_USCOREassertionStatusReport class.
*/

/**
@fn uddi2__get_USCOREassertionStatusReport::uddi2__get_USCOREassertionStatusReport(struct soap *soap, const char *completionStatus)
@brief
@param[in] soap gSOAP context
@param[in] completionStatus completion status string

Creates an instance of the get_USCOREassertionStatusReport class using the
specified completion status.
*/

/**
@fn uddi2__assertionStatusReport *uddi2__get_USCOREassertionStatusReport::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__assertionStatusReport object or NULL on error

Send a request to a UDDI service to get the status of current and outstanding
publisher assertions.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_authToken
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__get_USCOREauthToken
@brief

Represents a request to obtain an authorization token, which is represented by
the uddi2__authToken class.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137739
*/

/**
@fn uddi2__get_USCOREauthToken::uddi2__get_USCOREauthToken(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the get_USCOREauthToken class.
*/

/**
@fn uddi2__get_USCOREauthToken::uddi2__get_USCOREauthToken(struct soap *soap, const char *userid, const char *passwd)
@brief
@param[in] soap gSOAP context
@param[in] userid the user ID string
@param[in] passwd the password string

Creates an instance of the get_USCOREauthToken class using the specified user
ID and password.
*/

/**
@fn uddi2__authToken *uddi2__get_USCOREauthToken::send(const char *endpoint)
@brief
@param[in] endpoint URL of the UDDI server
@return A pointer to a uddi2__authToken object or NULL on error

Send a request to a UDDI service to get a authorization token given a user
ID and password.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_publisherAssertions
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__get_USCOREpublisherAssertions
@brief

Represents a request to obtain the full set of publisher assertions that are
associated with an individual publisher.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137740
*/

/**
@fn uddi2__get_USCOREpublisherAssertions::uddi2__get_USCOREpublisherAssertions(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the get_USCOREpublisherAssetions class.
*/

/**
@fn uddi2__publisherAssertions *uddi2__get_USCOREpublisherAssertions::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__publisherAssertions object or NULL on error

Send a request to a UDDI service to get publisher assertions.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_registeredInfo
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__get_USCOREregisteredInfo
@brief

Represents a request to get all registered business entities and tModels
controlled by an individual.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137741
*/

/**
@fn uddi2__get_USCOREregisteredInfo::uddi2__get_USCOREregisteredInfo(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the get_USCOREregisteredInfo class.
*/

/**
@fn uddi2__registeredInfo *uddi2__get_USCOREregisteredInfo::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__registeredInfo object or NULL on error

Send a request to a UDDI service to get all registered business entities and
tModels controlled by an individual.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:save_binding
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__save_USCOREbinding
@brief

Represents a request to post binding information on a UDDI server.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137742
*/

/**
@fn uddi2__save_USCOREbinding::uddi2__save_USCOREbinding(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the save_USCOREbinding class.
*/

/**
@fn uddi2__save_USCOREbinding::uddi2__save_USCOREbinding(struct soap *soap, uddi2__bindingTemplate &bindingTemplate)
@brief
@param[in] soap gSOAP context
@param[in] bindingTemplate binding template

Creates an instance of the save_USCOREbinding class using the specified binding
template.
*/

/**
@fn uddi2__save_USCOREbinding::uddi2__save_USCOREbinding(struct soap *soap, std::vector<uddi2__bindingTemplate*> bindingTemplates)
@brief
@param[in] soap gSOAP context
@param[in] bindingTemplates collection of binding templates

Creates an instance of the save_USCOREbinding class using the specified binding
templates.
*/

/**
@fn uddi2__bindingDetail *uddi2__save_USCOREbinding::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__bindingDetail object or NULL on error

Send a request to a UDDI server to post binding information on the server.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:save_business
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__save_USCOREbusiness
@brief

Represents a request to post business information on a UDDI server.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137743
*/

/**
@fn uddi2__save_USCOREbusiness::uddi2__save_USCOREbusiness(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the save_USCOREbusiness class.
*/

/**
@fn uddi2__save_USCOREbusiness::uddi2__save_USCOREbusiness(struct soap *soap, uddi2__businessEntity &businessEntity)
@brief
@param[in] soap gSOAP context
@param[in] businessEntity business entity

Creates an instance of the save_USCOREbusiness class using the specified
business entity.
*/

/**
@fn uddi2__save_USCOREbusiness::uddi2__save_USCOREbusiness(struct soap *soap, std::vector<uddi2__businessEntity*> businessEntities)
@brief
@param[in] soap gSOAP context
@param[in] businessEntities collection of business entities

Creates an instance of the save_USCOREbusiness class using the specified
business entities.
*/

/**
@fn uddi2__businessDetail *uddi2__save_USCOREbusiness::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__businessDetail object or NULL on error

Send a request to a UDDI server to post business information on the server.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:save_service
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__save_USCOREservice
@brief

Represents a request to post service information on a UDDI server.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137744
*/

/**
@fn uddi2__save_USCOREservice::uddi2__save_USCOREservice(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the save_USCOREservice class.
*/

/**
@fn uddi2__save_USCOREservice::uddi2__save_USCOREservice(struct soap *soap, uddi2__businessService &businessService)
@brief
@param[in] soap gSOAP context
@param[in] businessService business service

Creates an instance of the save_USCOREservice class using the specified
service.
*/

/**
@fn uddi2__save_USCOREservice::uddi2__save_USCOREservice(struct soap *soap, std::vector<uddi2__businessService*> businessServices)
@brief
@param[in] soap gSOAP context
@param[in] businessServices collection of business services

Creates an instance of the save_USCOREservice class using the specified
services.
*/

/**
@fn uddi2__serviceDetail *uddi2__save_USCOREservice::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__serviceDetail object or NULL on error

Send a request to a UDDI server to post service information on the server.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:save_tModel
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__save_USCOREtModel
@brief

Represents a request to post tModel information on a UDDI server.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137745
*/

/**
@fn uddi2__save_USCOREtModel::uddi2__save_USCOREtModel(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the save_USCOREtModel class.
*/

/**
@fn uddi2__save_USCOREtModel::uddi2__save_USCOREtModel(struct soap *soap, uddi2__tModel &tModel)
@brief
@param[in] soap gSOAP context
@param[in] tModel a tModel

Creates an instance of the save_USCOREtModel class using the specified
tModel.
*/

/**
@fn uddi2__save_USCOREtModel::uddi2__save_USCOREtModel(struct soap *soap, std::vector<uddi2__tModel*> tModels)
@brief
@param[in] soap gSOAP context
@param[in] tModels collection of tModels

Creates an instance of the save_USCOREtModel class using the specified
tModels.
*/

/**
@fn uddi2__tModelDetail *uddi2__save_USCOREtModel::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__tModelDetail object or NULL on error

Send a request to a UDDI server to post tModel information on the server.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:set_publisherAssertions
//
////////////////////////////////////////////////////////////////////////////////

/**
@class uddi2__set_USCOREpublisherAssertions
@brief

Represents a request to modify the existing publisher assertions for an
individual publisher.

See @url http://uddi.org/pubs/ProgrammersAPI_v2.htm#_Toc25137746
*/

/**
@fn uddi2__set_USCOREpublisherAssertions::uddi2__set_USCOREpublisherAssertions(struct soap *soap)
@brief
@param[in] soap gSOAP context

Creates an instance of the set_USCOREpublisherAssertions class.
*/

/**
@fn uddi2__set_USCOREpublisherAssertions::uddi2__set_USCOREpublisherAssertions(struct soap *soap, std::vector<uddi2__publisherAssertion*> publisherAssertions)
@brief
@param[in] soap gSOAP context
@param[in] publisherAssertions collection of publisher assertions

Creates an instance of the set_USCOREpublisherAssertions class using the
specified collection of publisher assertions.
*/

/**
@fn uddi2__publisherAssertions *uddi2__set_USCOREpublisherAssertions::send(const char *endpoint, char *authInfo)
@brief
@param[in] endpoint URL of the UDDI server
@param[in] authInfo authorization token provided by the UDDI server
@return A pointer to a uddi2__publisherAssertions object or NULL on error

Send a request to a UDDI server to post tModel information on the server.
*/

////////////////////////////////////////////////////////////////////////////////
//
//	Code Examples
//
////////////////////////////////////////////////////////////////////////////////

/**
@page example1 Code Example 1: Finding a service

This example shows you how to find Web services. In this case, the example
finds Web services with names that begin with the word "magic".

@code
#include "inqH.h"

int main(int argc, char **argv)
{ 
  char *search_string = "magic";

  if (argc > 1)
    search_string = argv[1];

  // Create a gSOAP context
  struct soap *soap = soap_new();

  // Create an object to find a business
  uddi2__find_USCOREservice fs(soap, search_string);

  // Send the request
  uddi2__serviceList *sl = fs.send("http://uddi.xmethods.net/inquire");

  // Check if result is OK
  if (!sl)
    soap_print_fault(soap, stderr);

  // If OK, report the service name(s) and unique identification keys
  else if (sl->serviceInfos)
  {
    std::cout << "Search results on " << search_string << ":" << std::endl << std::endl;

    for (std::vector<uddi2__serviceInfo*>::const_iterator si = sl->serviceInfos->serviceInfo.begin(); si != sl->serviceInfos->serviceInfo.end(); ++si)
    {
      // Report serviceKey and businessKey
      std::cout << "serviceKey=" << (*si)->serviceKey << std::endl << "businessKey=" << (*si)->businessKey << std::endl;

      // Report names
      for (std::vector<uddi2__name*>::const_iterator n = (*si)->name.begin(); n != (*si)->name.end(); ++n)
        std::cout << "name=" << (*n)->__item << std::endl;

      std::cout << std::endl;
    }
  }

  // Remove deserialized objects
  soap_destroy(soap);

  // Remove temporary data
  soap_end(soap);

  // Detach and free context
  soap_done(soap);
  free(soap);

  return 0;
}
@endcode

To compile:
- wsdl2h -tuddi2-typemap.dat inquire_v2.wsdl
- soapcpp2 -I.. -pinq inquire_v2.h
- g++ -DWITH_NONAMESPACES -I.. -o example1 example1.cpp inquire_v2.cpp inqC.cpp inqClient.cpp ../stdsoap2.cpp
*/

/**
@page example2 Code Example 2: Finding a business

This example shows you how to find a business from a UDDI server.

@code
#include "inqH.h"

int main(int argc, char **argv)
{ 
  char *search_string = "xmethods";

  if (argc > 1)
    search_string = argv[1];

  // Create a gSOAP context
  struct soap *soap = soap_new();

  // Create an object to find a business
  uddi2__find_USCOREbusiness fb(soap, search_string);

  // Send the request
  uddi2__businessList *bl = fb.send("http://uddi.xmethods.net/inquire");

  // Check if result is OK
  if (!bl)
    soap_print_fault(soap, stderr);

  // If OK, report the business name(s) and unique identification keys
  else if (bl->businessInfos)
  {
    std::cout << "Search results on " << search_string << ":" << std::endl << std::endl;

    for (std::vector<uddi2__businessInfo*>::const_iterator bi = bl->businessInfos->businessInfo.begin(); bi != bl->businessInfos->businessInfo.end(); ++bi)
    {
      // Report businessKey
      std::cout << "businessKey=" << (*bi)->businessKey << std::endl;

      // Report names
      for (std::vector<uddi2__name*>::const_iterator n = (*bi)->name.begin(); n != (*bi)->name.end(); ++n)
        std::cout << "name=" << (*n)->__item << std::endl;

      std::cout << std::endl;
    }
  }

  // Remove deserialized objects
  soap_destroy(soap);

  // Remove temporary data
  soap_end(soap);

  // Detach and free context
  soap_done(soap);
  free(soap);

  return 0;
}
@endcode

To compile:
- wsdl2h -tuddi2-typemap.dat inquire_v2.wsdl
- soapcpp2 -I.. -pinq inquire_v2.h
- g++ -DWITH_NONAMESPACES -I.. -o example2 example2.cpp inquire_v2.cpp inqC.cpp inqClient.cpp ../stdsoap2.cpp
*/
/**
@page example3 Code Example 3: Publishing a WSDL and service on XMethods

This example shows you how to publish a Web service. In this case, the example
template code obtains an authorization token, saves the tModel with the WSDL
URL in the server, and saves the business service information in the server.

@code
#include "pubH.h"

const char *server = "https://uddi.xmethods.net/publish";

const char *userid = "..."; // user ID to access UDDI server
const char *passwd = "..."; // password to access UDDI server

int main(int argc, char **argv)
{ 
  // Create a gSOAP context
  struct soap *soap = soap_new();

  // Setup SSL context (optional) to verify server's credentials
  if (soap_ssl_client_context(soap, SOAP_SSL_DEFAULT, NULL, NULL, "cacerts.pem", NULL, NULL))
  { 
    soap_print_fault(soap, stderr);
    exit(1);
  }

  // Step 1: Get an authorization token from the UDDI server
  uddi2__get_USCOREauthToken get_authToken(soap, userid, passwd);
  uddi2__authToken *authToken = get_authToken.send(server);

  // Check if authorized
  if (!authToken)
  {
    soap_print_fault(soap, stderr);
    exit(1);
  }

  // Authorization info provided by server for this session
  char *authInfo = authToken->authInfo;

  // Step 2: Create a tModel for the WSDL to be published
  uddi2__tModel tModel;
  tModel.soap_default(soap);

  // Create the tModel and service name
  tModel.name = soap_new_uddi2__name(soap, -1);
  tModel.name->__item = "...";
  tModel.name->xml__lang_ = "en";

  // Create XMethods description elements (see http://www.xmethods.net/ve2/UDDI.po)
  uddi2__description *description = soap_new_uddi2__description(soap, 6);
  description[0].__item = "SHORT DESCRIPTION: ...";
  description[0].xml__lang_ = "en";
  description[1].__item = "SHORT DESCRIPTION: ...";
  description[1].xml__lang_ = "en";
  description[2].__item = "USAGE NOTES: ...";
  description[2].xml__lang_ = "en";
  description[3].__item = "HOMEPAGE URL: ...";
  description[3].xml__lang_ = "en";
  description[4].__item = "CONTACT EMAIL: ...";
  description[4].xml__lang_ = "en";
  description[5].__item = "IMPLEMENTATION: ...";
  description[5].xml__lang_ = "en";

  // Add the four description elements to the tModel
  tModel.description.push_back(description + 0);
  tModel.description.push_back(description + 1);
  tModel.description.push_back(description + 2);
  tModel.description.push_back(description + 4);

  // Add an overviewDoc element with description and overviewURL
  tModel.overviewDoc = soap_new_uddi2__overviewDoc(soap, -1);
  tModel.overviewDoc->soap_default(soap);
  tModel.overviewDoc->description.push_back(soap_new_uddi2__description(soap, -1));
  tModel.overviewDoc->description[0]->__item = "WSDL source document";
  tModel.overviewDoc->description[0]->xml__lang_ = "en";
  tModel.overviewDoc->overviewURL = "http://.../my.wsdl#bindingName";

  // Omit identifier bag
  tModel.identifierBag = NULL;

  // Add a category with a WSDL-specific keyedReference
  tModel.categoryBag = soap_new_uddi2__categoryBag(soap, -1);
  tModel.categoryBag->soap_default(soap);
  tModel.categoryBag->keyedReference.push_back(soap_new_uddi2__keyedReference(soap, -1));
  tModel.categoryBag->keyedReference[0]->tModelKey = "...";
  tModel.categoryBag->keyedReference[0]->keyName = "uddi-org:types";
  tModel.categoryBag->keyedReference[0]->keyValue = "wsdlSpec";

  tModel.authorizedName = "...";
  tModel.operator_ = "...";
  tModel.tModelKey = "...";

  // Save the tModel
  uddi2__save_USCOREtModel save_tModel(soap, tModel);
  uddi2__tModelDetail *tModelDetail = save_tModel.send(server, authInfo);

  // Step 3: Create a new service to be published
  uddi2__businessService service;
  service.soap_default(soap);

  // Service name is the tModel name (XMethods)
  service.name.push_back(tModel.name);

  // Add two description elements to the service
  service.description.push_back(description + 4);
  service.description.push_back(description + 5);

  // Create binding template
  uddi2__bindingTemplate bindingTemplate;
  bindingTemplate.soap_default(soap);
  bindingTemplate.tModelInstanceDetails = soap_new_uddi2__tModelInstanceDetails(soap, -1);
  bindingTemplate.tModelInstanceDetails->tModelInstanceInfo.push_back(soap_new_uddi2__tModelInstanceInfo(soap, -1));
  bindingTemplate.tModelInstanceDetails->tModelInstanceInfo[0]->instanceDetails = NULL;
  bindingTemplate.tModelInstanceDetails->tModelInstanceInfo[0]->tModelKey = tModel.tModelKey;
  bindingTemplate.accessPoint = soap_new_uddi2__accessPoint(soap, -1);
  bindingTemplate.accessPoint->__item = "...";
  bindingTemplate.accessPoint->URLType = uddi2__URLType__http;
  bindingTemplate.hostingRedirector = NULL;
  bindingTemplate.serviceKey = "...";
  bindingTemplate.bindingKey = "...";

  // Add binding Template to service
  service.bindingTemplates = soap_new_uddi2__bindingTemplates(soap, -1);
  service.bindingTemplates->soap_default(soap);
  service.bindingTemplates->bindingTemplate.push_back(&bindingTemplate);

  service.categoryBag = NULL;
  service.serviceKey = "...";
  service.businessKey = "...";

  // Save the service
  uddi2__save_USCOREservice save_service(soap, service);
  uddi2__serviceDetail *serviceDetail = save_service.send(server, authInfo);

  // Step 4: Discard authorization token
  uddi2__discard_USCOREauthToken discard_authToken(soap, authInfo);
  uddi2__dispositionReport *dispositionReport = discard_authToken.send(server);

  // Remove deserialized objects
  soap_destroy(soap);

  // Remove temporary data
  soap_end(soap);

  // Detach and free context
  soap_done(soap);
  free(soap);

  return 0;
}
@endcode
To compile:
- wsdl2h -tuddi2-typemap.dat publish_v2.wsdl
- soapcpp2 -I.. -ppub publish_v2.h
- g++ -DWITH_OPENSSL -DWITH_NONAMESPACES -I.. -o example3 example3.cpp publish_v2.cpp pubC.cpp pubClient.cpp ../stdsoap2.cpp
*/
/* uddi_v2.h
   Generated by wsdl2h 1.2.0 from inquire_v2.wsdl publish_v2.wsdl and uddi2-typemap.dat
   2005-04-24 00:29:41 GMT
   Copyright (C) 2001-2005 Robert van Engelen, Genivia Inc. All Rights Reserved.
   This part of the software is released under one of the following licenses:
   GPL or Genivia's license for commercial use.
*/

/* NOTE:

 - Compile this file with soapcpp2 to complete the code generation process.
 - Use wsdl2h option -l to view the software license terms.
 - Use wsdl2h options -c and -s to generate pure C code or C++ code without STL.
 - To build with STL, stlvector.h from the gSOAP distribution must be in the
   current directory. Or use soapcpp2 option -I<path> with path to stlvector.h.
 - Use typemap.dat to control schema namespace bindings and type mappings.
   It is strongly recommended to customize the names of the namespace prefixes
   generated by wsdl2h. To do so, modify the prefix bindings in the Namespaces
   section below and add the modified lines to typemap.dat to rerun wsdl2h.
 - Use Doxygen (www.doxygen.org) to browse this file.

*/

/******************************************************************************\
 *                                                                            *
 *                                                                            *
 *                                                                            *
\******************************************************************************/

//gsoapopt w

#import "stlvector.h"

/******************************************************************************\
 *                                                                            *
 * Schema Namespaces                                                          *
 *                                                                            *
\******************************************************************************/


/* NOTE:

It is strongly recommended to customize the names of the namespace prefixes
generated by wsdl2h. To do so, modify the prefix bindings below and add the
modified lines to typemap.dat to rerun wsdl2h:

inq2 = urn:uddi-org:inquiry_v2
uddi2 = urn:uddi-org:api_v2
pub2 = urn:uddi-org:publication_v2

*/

//gsoap inq2  schema namespace:	urn:uddi-org:inquiry_v2
//gsoap uddi2 schema namespace:	urn:uddi-org:api_v2
//gsoap pub2  schema namespace:	urn:uddi-org:publication_v2
//gsoap inq2  schema form:	unqualified
//gsoap uddi2 schema elementForm:	qualified
//gsoap uddi2 schema attributeForm:	unqualified
//gsoap pub2  schema form:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/


/// Primitive built-in type "xs:NMTOKEN"
typedef char* xsd__NMTOKEN;

/// Built-in attribute "xml:lang"
typedef char* xml__lang;

class uddi2__accessPoint;
class uddi2__address;
class uddi2__addressLine;
class uddi2__assertionStatusItem;
class uddi2__bindingTemplate;
class uddi2__bindingTemplates;
class uddi2__businessEntity;
class uddi2__businessEntityExt;
class uddi2__businessInfo;
class uddi2__businessInfos;
class uddi2__businessService;
class uddi2__businessServices;
class uddi2__categoryBag;
class uddi2__contact;
class uddi2__contacts;
class uddi2__description;
class uddi2__discoveryURL;
class uddi2__discoveryURLs;
class uddi2__dispositionReport;
class uddi2__email;
class uddi2__errInfo;
class uddi2__findQualifiers;
class uddi2__hostingRedirector;
class uddi2__identifierBag;
class uddi2__instanceDetails;
class uddi2__keyedReference;
class uddi2__keysOwned;
class uddi2__name;
class uddi2__overviewDoc;
class uddi2__phone;
class uddi2__publisherAssertion;
class uddi2__relatedBusinessInfo;
class uddi2__relatedBusinessInfos;
class uddi2__result;
class uddi2__serviceInfo;
class uddi2__serviceInfos;
class uddi2__sharedRelationships;
class uddi2__tModel;
class uddi2__tModelBag;
class uddi2__tModelInfo;
class uddi2__tModelInfos;
class uddi2__tModelInstanceDetails;
class uddi2__tModelInstanceInfo;
class uddi2__add_USCOREpublisherAssertions;
class uddi2__delete_USCOREbinding;
class uddi2__delete_USCOREbusiness;
class uddi2__delete_USCOREpublisherAssertions;
class uddi2__delete_USCOREservice;
class uddi2__delete_USCOREtModel;
class uddi2__discard_USCOREauthToken;
class uddi2__find_USCOREbinding;
class uddi2__find_USCOREbusiness;
class uddi2__find_USCORErelatedBusinesses;
class uddi2__find_USCOREservice;
class uddi2__find_USCOREtModel;
class uddi2__get_USCOREassertionStatusReport;
class uddi2__get_USCOREauthToken;
class uddi2__get_USCOREbindingDetail;
class uddi2__get_USCOREbusinessDetail;
class uddi2__get_USCOREbusinessDetailExt;
class uddi2__get_USCOREpublisherAssertions;
class uddi2__get_USCOREregisteredInfo;
class uddi2__get_USCOREserviceDetail;
class uddi2__get_USCOREtModelDetail;
class uddi2__save_USCOREbinding;
class uddi2__save_USCOREbusiness;
class uddi2__save_USCOREservice;
class uddi2__save_USCOREtModel;
class uddi2__set_USCOREpublisherAssertions;
class uddi2__validate_USCOREvalues;
class uddi2__assertionStatusReport;
class uddi2__authToken;
class uddi2__bindingDetail;
class uddi2__businessDetail;
class uddi2__businessDetailExt;
class uddi2__businessList;
class uddi2__publisherAssertions;
class uddi2__registeredInfo;
class uddi2__relatedBusinessesList;
class uddi2__serviceDetail;
class uddi2__serviceList;
class uddi2__tModelDetail;
class uddi2__tModelList;

/// Schema "urn:uddi-org:api_v2":bindingKey simpleType restriction of xs:string

typedef char* uddi2__bindingKey;

/// Schema "urn:uddi-org:api_v2":businessKey simpleType restriction of xs:string

typedef char* uddi2__businessKey;

/// Schema "urn:uddi-org:api_v2":serviceKey simpleType restriction of xs:string

typedef char* uddi2__serviceKey;

/// Schema "urn:uddi-org:api_v2":tModelKey simpleType restriction of xs:string

typedef char* uddi2__tModelKey;

/// Schema "urn:uddi-org:api_v2":direction simpleType restriction of xs:NMTOKEN

/// Note: enum values are prefixed with 'uddi2__direction' to avoid name clashes, please use wsdl2h option -e to omit this prefix
enum uddi2__direction
{	uddi2__direction__fromKey,	///< xs:NMTOKEN value="fromKey"
	uddi2__direction__toKey,	///< xs:NMTOKEN value="toKey"
};

/// Schema "urn:uddi-org:api_v2":truncated simpleType restriction of xs:NMTOKEN

/// Note: enum values are prefixed with 'uddi2__truncated' to avoid name clashes, please use wsdl2h option -e to omit this prefix
enum uddi2__truncated
{	uddi2__truncated__true_,	///< xs:NMTOKEN value="true"
	uddi2__truncated__false_,	///< xs:NMTOKEN value="false"
};

/// Schema "urn:uddi-org:api_v2":URLType simpleType restriction of xs:NMTOKEN

/// Note: enum values are prefixed with 'uddi2__URLType' to avoid name clashes, please use wsdl2h option -e to omit this prefix
enum uddi2__URLType
{	uddi2__URLType__mailto,	///< xs:NMTOKEN value="mailto"
	uddi2__URLType__http,	///< xs:NMTOKEN value="http"
	uddi2__URLType__https,	///< xs:NMTOKEN value="https"
	uddi2__URLType__ftp,	///< xs:NMTOKEN value="ftp"
	uddi2__URLType__fax,	///< xs:NMTOKEN value="fax"
	uddi2__URLType__phone,	///< xs:NMTOKEN value="phone"
	uddi2__URLType__other,	///< xs:NMTOKEN value="other"
};

/// Schema "urn:uddi-org:api_v2":keyType simpleType restriction of xs:NMTOKEN

/// Note: enum values are prefixed with 'uddi2__keyType' to avoid name clashes, please use wsdl2h option -e to omit this prefix
enum uddi2__keyType
{	uddi2__keyType__businessKey,	///< xs:NMTOKEN value="businessKey"
	uddi2__keyType__tModelKey,	///< xs:NMTOKEN value="tModelKey"
	uddi2__keyType__serviceKey,	///< xs:NMTOKEN value="serviceKey"
	uddi2__keyType__bindingKey,	///< xs:NMTOKEN value="bindingKey"
};

/// Schema urn:uddi-org:api_v2:"address"

class uddi2__address
{ public:
/// Vector of uddi2__addressLine* with length 0..unbounded
    std::vector<uddi2__addressLine*    > addressLine                    0;
/// Attribute useType of type xs:string
   @char*                                useType                        0;	///< Optional attribute
/// Attribute sortCode of type xs:string
   @char*                                sortCode                       0;	///< Optional attribute
/// Attribute tModelKey of type "urn:uddi-org:api_v2":tModelKey
   @uddi2__tModelKey                     tModelKey                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"assertionStatusItem"

class uddi2__assertionStatusItem
{ public:
/// Element reference "urn:uddi-org:api_v2":fromKey
    uddi2__businessKey                   fromKey                        1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":toKey
    uddi2__businessKey                   toKey                          1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":keyedReference
    uddi2__keyedReference*               keyedReference                 1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":keysOwned
    uddi2__keysOwned*                    keysOwned                      1;	///< Required element
/// Attribute completionStatus of type xs:string
   @char*                                completionStatus               1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"bindingTemplate"

class uddi2__bindingTemplate
{ public:
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Element reference "urn:uddi-org:api_v2":tModelInstanceDetails
    uddi2__tModelInstanceDetails*        tModelInstanceDetails          1;	///< Required element
// CHOICE OF ELEMENTS:
/// Element reference "urn:uddi-org:api_v2":accessPoint
    uddi2__accessPoint*                  accessPoint                    1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":hostingRedirector
    uddi2__hostingRedirector*            hostingRedirector              1;	///< Required element
// END CHOICE
/// Attribute serviceKey of type "urn:uddi-org:api_v2":serviceKey
   @uddi2__serviceKey                    serviceKey                     0;	///< Optional attribute
/// Attribute bindingKey of type "urn:uddi-org:api_v2":bindingKey
   @uddi2__bindingKey                    bindingKey                     1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"bindingTemplates"

class uddi2__bindingTemplates
{ public:
/// Vector of uddi2__bindingTemplate* with length 0..unbounded
    std::vector<uddi2__bindingTemplate*> bindingTemplate                0;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"businessEntity"

class uddi2__businessEntity
{ public:
/// Element reference "urn:uddi-org:api_v2":discoveryURLs
    uddi2__discoveryURLs*                discoveryURLs                  0;	///< Optional element
/// Vector of uddi2__name* with length 0..unbounded
    std::vector<uddi2__name*           > name                           1;
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Element reference "urn:uddi-org:api_v2":contacts
    uddi2__contacts*                     contacts                       0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":businessServices
    uddi2__businessServices*             businessServices               0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":identifierBag
    uddi2__identifierBag*                identifierBag                  0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":categoryBag
    uddi2__categoryBag*                  categoryBag                    0;	///< Optional element
/// Attribute businessKey of type "urn:uddi-org:api_v2":businessKey
   @uddi2__businessKey                   businessKey                    1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      0;	///< Optional attribute
/// Attribute authorizedName of type xs:string
   @char*                                authorizedName                 0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"businessEntityExt"

class uddi2__businessEntityExt
{ public:
/// Element reference "urn:uddi-org:api_v2":businessEntity
    uddi2__businessEntity*               businessEntity                 1;	///< Required element
// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
//       Schema extensibility is user-definable
//       Consult the protocol documentation to insert declarations here:
    _XML                                 __any                         ;	///< Catch any element content in XML string
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"businessInfo"

class uddi2__businessInfo
{ public:
/// Vector of uddi2__name* with length 0..unbounded
    std::vector<uddi2__name*           > name                           1;
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Element reference "urn:uddi-org:api_v2":serviceInfos
    uddi2__serviceInfos*                 serviceInfos                   1;	///< Required element
/// Attribute businessKey of type "urn:uddi-org:api_v2":businessKey
   @uddi2__businessKey                   businessKey                    1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"businessInfos"

class uddi2__businessInfos
{ public:
/// Vector of uddi2__businessInfo* with length 0..unbounded
    std::vector<uddi2__businessInfo*   > businessInfo                   0;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"businessService"

class uddi2__businessService
{ public:
/// Vector of uddi2__name* with length 0..unbounded
    std::vector<uddi2__name*           > name                           0;
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Element reference "urn:uddi-org:api_v2":bindingTemplates
    uddi2__bindingTemplates*             bindingTemplates               0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":categoryBag
    uddi2__categoryBag*                  categoryBag                    0;	///< Optional element
/// Attribute serviceKey of type "urn:uddi-org:api_v2":serviceKey
   @uddi2__serviceKey                    serviceKey                     1;	///< Required attribute
/// Attribute businessKey of type "urn:uddi-org:api_v2":businessKey
   @uddi2__businessKey                   businessKey                    0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"businessServices"

class uddi2__businessServices
{ public:
/// Vector of uddi2__businessService* with length 0..unbounded
    std::vector<uddi2__businessService*> businessService                0;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"categoryBag"

class uddi2__categoryBag
{ public:
/// Vector of uddi2__keyedReference* with length 0..unbounded
    std::vector<uddi2__keyedReference* > keyedReference                 1;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"contact"

class uddi2__contact
{ public:
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Element reference "urn:uddi-org:api_v2":personName
    char*                                personName                     1;	///< Required element
/// Vector of uddi2__phone* with length 0..unbounded
    std::vector<uddi2__phone*          > phone                          0;
/// Vector of uddi2__email* with length 0..unbounded
    std::vector<uddi2__email*          > email                          0;
/// Vector of uddi2__address* with length 0..unbounded
    std::vector<uddi2__address*        > address                        0;
/// Attribute useType of type xs:string
   @char*                                useType                        0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"contacts"

class uddi2__contacts
{ public:
/// Vector of uddi2__contact* with length 0..unbounded
    std::vector<uddi2__contact*        > contact                        1;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"discoveryURLs"

class uddi2__discoveryURLs
{ public:
/// Vector of uddi2__discoveryURL* with length 0..unbounded
    std::vector<uddi2__discoveryURL*   > discoveryURL                   1;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"dispositionReport"

class uddi2__dispositionReport
{ public:
/// Vector of uddi2__result* with length 0..unbounded
    std::vector<uddi2__result*         > result                         1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"findQualifiers"

class uddi2__findQualifiers
{ public:
/// Vector of char* with length 0..unbounded
    std::vector<char*                  > findQualifier                  0;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"hostingRedirector"

class uddi2__hostingRedirector
{ public:
/// Attribute bindingKey of type "urn:uddi-org:api_v2":bindingKey
   @uddi2__bindingKey                    bindingKey                     1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"identifierBag"

class uddi2__identifierBag
{ public:
/// Vector of uddi2__keyedReference* with length 0..unbounded
    std::vector<uddi2__keyedReference* > keyedReference                 1;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"instanceDetails"

class uddi2__instanceDetails
{ public:
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Element reference "urn:uddi-org:api_v2":overviewDoc
    uddi2__overviewDoc*                  overviewDoc                    0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":instanceParms
    char*                                instanceParms                  0;	///< Optional element
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"keyedReference"

class uddi2__keyedReference
{ public:
/// Attribute tModelKey of type "urn:uddi-org:api_v2":tModelKey
   @uddi2__tModelKey                     tModelKey                      0;	///< Optional attribute
/// Attribute keyName of type xs:string
   @char*                                keyName                        0;	///< Optional attribute
/// Attribute keyValue of type xs:string
   @char*                                keyValue                       1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"keysOwned"

class uddi2__keysOwned
{ public:
/// Element reference "urn:uddi-org:api_v2":fromKey
    uddi2__businessKey                   fromKey                        0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":toKey
    uddi2__businessKey                   toKey                          0;	///< Optional element
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"overviewDoc"

class uddi2__overviewDoc
{ public:
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Element reference "urn:uddi-org:api_v2":overviewURL
    char*                                overviewURL                    0;	///< Optional element
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"publisherAssertion"

class uddi2__publisherAssertion
{ public:
/// Element reference "urn:uddi-org:api_v2":fromKey
    uddi2__businessKey                   fromKey                        1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":toKey
    uddi2__businessKey                   toKey                          1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":keyedReference
    uddi2__keyedReference*               keyedReference                 1;	///< Required element
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"relatedBusinessInfo"

class uddi2__relatedBusinessInfo
{ public:
/// Element reference "urn:uddi-org:api_v2":businessKey
    uddi2__businessKey                   businessKey                    1;	///< Required element
/// Vector of uddi2__name* with length 0..unbounded
    std::vector<uddi2__name*           > name                           1;
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Vector of uddi2__sharedRelationships* with length 0..2
    std::vector<uddi2__sharedRelationships*> sharedRelationships            1:2;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"relatedBusinessInfos"

class uddi2__relatedBusinessInfos
{ public:
/// Vector of uddi2__relatedBusinessInfo* with length 0..unbounded
    std::vector<uddi2__relatedBusinessInfo*> relatedBusinessInfo            0;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"result"

class uddi2__result
{ public:
/// Element reference "urn:uddi-org:api_v2":errInfo
    uddi2__errInfo*                      errInfo                        0;	///< Optional element
/// Attribute keyType of type "urn:uddi-org:api_v2":keyType
   @enum uddi2__keyType*                 keyType                        0;	///< Optional attribute
/// Attribute errno of type xs:int
   @int                                  errno_                         1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"serviceInfo"

class uddi2__serviceInfo
{ public:
/// Vector of uddi2__name* with length 0..unbounded
    std::vector<uddi2__name*           > name                           0;
/// Attribute serviceKey of type "urn:uddi-org:api_v2":serviceKey
   @uddi2__serviceKey                    serviceKey                     1;	///< Required attribute
/// Attribute businessKey of type "urn:uddi-org:api_v2":businessKey
   @uddi2__businessKey                   businessKey                    1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"serviceInfos"

class uddi2__serviceInfos
{ public:
/// Vector of uddi2__serviceInfo* with length 0..unbounded
    std::vector<uddi2__serviceInfo*    > serviceInfo                    0;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"sharedRelationships"

class uddi2__sharedRelationships
{ public:
/// Vector of uddi2__keyedReference* with length 0..unbounded
    std::vector<uddi2__keyedReference* > keyedReference                 1;
/// Attribute direction of type "urn:uddi-org:api_v2":direction
   @enum uddi2__direction                direction                      1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"tModel"

class uddi2__tModel
{ public:
/// Element reference "urn:uddi-org:api_v2":name
    uddi2__name*                         name                           1;	///< Required element
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Element reference "urn:uddi-org:api_v2":overviewDoc
    uddi2__overviewDoc*                  overviewDoc                    0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":identifierBag
    uddi2__identifierBag*                identifierBag                  0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":categoryBag
    uddi2__categoryBag*                  categoryBag                    0;	///< Optional element
/// Attribute tModelKey of type "urn:uddi-org:api_v2":tModelKey
   @uddi2__tModelKey                     tModelKey                      1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      0;	///< Optional attribute
/// Attribute authorizedName of type xs:string
   @char*                                authorizedName                 0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"tModelBag"

class uddi2__tModelBag
{ public:
/// Vector of uddi2__tModelKey with length 0..unbounded
    std::vector<uddi2__tModelKey       > tModelKey                      1;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"tModelInfo"

class uddi2__tModelInfo
{ public:
/// Element reference "urn:uddi-org:api_v2":name
    uddi2__name*                         name                           1;	///< Required element
/// Attribute tModelKey of type "urn:uddi-org:api_v2":tModelKey
   @uddi2__tModelKey                     tModelKey                      1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"tModelInfos"

class uddi2__tModelInfos
{ public:
/// Vector of uddi2__tModelInfo* with length 0..unbounded
    std::vector<uddi2__tModelInfo*     > tModelInfo                     0;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"tModelInstanceDetails"

class uddi2__tModelInstanceDetails
{ public:
/// Vector of uddi2__tModelInstanceInfo* with length 0..unbounded
    std::vector<uddi2__tModelInstanceInfo*> tModelInstanceInfo             0;
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"tModelInstanceInfo"

class uddi2__tModelInstanceInfo
{ public:
/// Vector of uddi2__description* with length 0..unbounded
    std::vector<uddi2__description*    > description                    0;
/// Element reference "urn:uddi-org:api_v2":instanceDetails
    uddi2__instanceDetails*              instanceDetails                0;	///< Optional element
/// Attribute tModelKey of type "urn:uddi-org:api_v2":tModelKey
   @uddi2__tModelKey                     tModelKey                      1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"add_publisherAssertions"

class uddi2__add_USCOREpublisherAssertions
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__publisherAssertion* with length 0..unbounded
    std::vector<uddi2__publisherAssertion*> publisherAssertion             1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__add_USCOREpublisherAssertions(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__add_USCOREpublisherAssertions(struct soap*, std::vector<uddi2__publisherAssertion*> publisherAssertions);
/// Member declared in uddi2-typemap.dat
    uddi2__dispositionReport* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"delete_binding"

class uddi2__delete_USCOREbinding
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__bindingKey with length 0..unbounded
    std::vector<uddi2__bindingKey      > bindingKey                     1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREbinding(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREbinding(struct soap*, const char *bindingKey);
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREbinding(struct soap*, std::vector<char*> bindingKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__dispositionReport* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"delete_business"

class uddi2__delete_USCOREbusiness
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__businessKey with length 0..unbounded
    std::vector<uddi2__businessKey     > businessKey                    1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREbusiness(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREbusiness(struct soap*, const char *businessKey);
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREbusiness(struct soap*, std::vector<char*> businessKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__dispositionReport* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"delete_publisherAssertions"

class uddi2__delete_USCOREpublisherAssertions
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__publisherAssertion* with length 0..unbounded
    std::vector<uddi2__publisherAssertion*> publisherAssertion             1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREpublisherAssertions(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREpublisherAssertions(struct soap*, std::vector<uddi2__publisherAssertion*> publisherAssertions);
/// Member declared in uddi2-typemap.dat
    uddi2__dispositionReport* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"delete_service"

class uddi2__delete_USCOREservice
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__serviceKey with length 0..unbounded
    std::vector<uddi2__serviceKey      > serviceKey                     1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREservice(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREservice(struct soap*, const char *serviceKey);
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREservice(struct soap*, std::vector<char*> serviceKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__dispositionReport* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"delete_tModel"

class uddi2__delete_USCOREtModel
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__tModelKey with length 0..unbounded
    std::vector<uddi2__tModelKey       > tModelKey                      1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREtModel(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREtModel(struct soap*, const char *tModelKey);
/// Member declared in uddi2-typemap.dat
    uddi2__delete_USCOREtModel(struct soap*, std::vector<char*> tModelKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__dispositionReport* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"discard_authToken"

class uddi2__discard_USCOREauthToken
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__discard_USCOREauthToken(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__discard_USCOREauthToken(struct soap*, const char *authInfo);
/// Member declared in uddi2-typemap.dat
    uddi2__dispositionReport* send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"find_binding"

class uddi2__find_USCOREbinding
{ public:
/// Element reference "urn:uddi-org:api_v2":findQualifiers
    uddi2__findQualifiers*               findQualifiers                 0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":tModelBag
    uddi2__tModelBag*                    tModelBag                      1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute maxRows of type xs:int
   @int*                                 maxRows                        0;	///< Optional attribute
/// Attribute serviceKey of type "urn:uddi-org:api_v2":serviceKey
   @uddi2__serviceKey                    serviceKey                     1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREbinding(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREbinding(struct soap*, const char *tModelKey);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREbinding(struct soap*, std::vector<char*> tModelKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__bindingDetail *send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"find_business"

class uddi2__find_USCOREbusiness
{ public:
/// Element reference "urn:uddi-org:api_v2":findQualifiers
    uddi2__findQualifiers*               findQualifiers                 0;	///< Optional element
/// Vector of uddi2__name* with length 0..unbounded
    std::vector<uddi2__name*           > name                           0;
/// Element reference "urn:uddi-org:api_v2":identifierBag
    uddi2__identifierBag*                identifierBag                  0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":categoryBag
    uddi2__categoryBag*                  categoryBag                    0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":tModelBag
    uddi2__tModelBag*                    tModelBag                      0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":discoveryURLs
    uddi2__discoveryURLs*                discoveryURLs                  0;	///< Optional element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute maxRows of type xs:int
   @int*                                 maxRows                        0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREbusiness(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREbusiness(struct soap*, const char *name);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREbusiness(struct soap*, std::vector<char*> tModelKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREbusiness(struct soap*, std::vector<uddi2__keyedReference*> keyedReferences);
/// Member declared in uddi2-typemap.dat
    uddi2__businessList *send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"find_relatedBusinesses"

class uddi2__find_USCORErelatedBusinesses
{ public:
/// Element reference "urn:uddi-org:api_v2":findQualifiers
    uddi2__findQualifiers*               findQualifiers                 0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":businessKey
    uddi2__businessKey                   businessKey                    1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":keyedReference
    uddi2__keyedReference*               keyedReference                 0;	///< Optional element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute maxRows of type xs:int
   @int*                                 maxRows                        0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCORErelatedBusinesses(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCORErelatedBusinesses(struct soap*, const char *businessKey);
/// Member declared in uddi2-typemap.dat
    uddi2__relatedBusinessesList *send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"find_service"

class uddi2__find_USCOREservice
{ public:
/// Element reference "urn:uddi-org:api_v2":findQualifiers
    uddi2__findQualifiers*               findQualifiers                 0;	///< Optional element
/// Vector of uddi2__name* with length 0..unbounded
    std::vector<uddi2__name*           > name                           0;
/// Element reference "urn:uddi-org:api_v2":categoryBag
    uddi2__categoryBag*                  categoryBag                    0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":tModelBag
    uddi2__tModelBag*                    tModelBag                      0;	///< Optional element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute maxRows of type xs:int
   @int*                                 maxRows                        0;	///< Optional attribute
/// Attribute businessKey of type "urn:uddi-org:api_v2":businessKey
   @uddi2__businessKey                   businessKey                    0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREservice(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREservice(struct soap*, const char *name);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREservice(struct soap*, std::vector<char*> tModelKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREservice(struct soap*, std::vector<uddi2__keyedReference*> keyedReferences);
/// Member declared in uddi2-typemap.dat
    uddi2__serviceList* send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"find_tModel"

class uddi2__find_USCOREtModel
{ public:
/// Element reference "urn:uddi-org:api_v2":findQualifiers
    uddi2__findQualifiers*               findQualifiers                 0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":name
    uddi2__name*                         name                           0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":identifierBag
    uddi2__identifierBag*                identifierBag                  0;	///< Optional element
/// Element reference "urn:uddi-org:api_v2":categoryBag
    uddi2__categoryBag*                  categoryBag                    0;	///< Optional element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute maxRows of type xs:int
   @int*                                 maxRows                        0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREtModel(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREtModel(struct soap*, const char *name);
/// Member declared in uddi2-typemap.dat
    uddi2__find_USCOREtModel(struct soap*, std::vector<uddi2__keyedReference*> keyedReferences);
/// Member declared in uddi2-typemap.dat
    uddi2__tModelList* send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"get_assertionStatusReport"

class uddi2__get_USCOREassertionStatusReport
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":completionStatus
    char*                                completionStatus               0;	///< Optional element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREassertionStatusReport(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREassertionStatusReport(struct soap*, const char *completionStatus);
/// Member declared in uddi2-typemap.dat
    uddi2__assertionStatusReport* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"get_authToken"

class uddi2__get_USCOREauthToken
{ public:
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute userID of type xs:string
   @char*                                userID                         1;	///< Required attribute
/// Attribute cred of type xs:string
   @char*                                cred                           1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREauthToken(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREauthToken(struct soap*, const char *userid, const char *passwd);
/// Member declared in uddi2-typemap.dat
    uddi2__authToken* send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"get_bindingDetail"

class uddi2__get_USCOREbindingDetail
{ public:
/// Vector of uddi2__bindingKey with length 0..unbounded
    std::vector<uddi2__bindingKey      > bindingKey                     1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREbindingDetail(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREbindingDetail(struct soap*, const char *bindingKey);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREbindingDetail(struct soap*, std::vector<char*> bindingKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__bindingDetail* send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"get_businessDetail"

class uddi2__get_USCOREbusinessDetail
{ public:
/// Vector of uddi2__businessKey with length 0..unbounded
    std::vector<uddi2__businessKey     > businessKey                    1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREbusinessDetail(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREbusinessDetail(struct soap*, const char *businessKey);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREbusinessDetail(struct soap*, std::vector<char*> businessKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__businessDetail* send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"get_businessDetailExt"

class uddi2__get_USCOREbusinessDetailExt
{ public:
/// Vector of uddi2__businessKey with length 0..unbounded
    std::vector<uddi2__businessKey     > businessKey                    1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREbusinessDetailExt(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREbusinessDetailExt(struct soap*, const char *businessKey);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREbusinessDetailExt(struct soap*, std::vector<char*> businessKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__businessDetailExt* send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"get_publisherAssertions"

class uddi2__get_USCOREpublisherAssertions
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREpublisherAssertions(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__publisherAssertions* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"get_registeredInfo"

class uddi2__get_USCOREregisteredInfo
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREregisteredInfo(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__registeredInfo* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"get_serviceDetail"

class uddi2__get_USCOREserviceDetail
{ public:
/// Vector of uddi2__serviceKey with length 0..unbounded
    std::vector<uddi2__serviceKey      > serviceKey                     1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREserviceDetail(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREserviceDetail(struct soap*, const char *serviceKey);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREserviceDetail(struct soap*, std::vector<char*> serviceKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__serviceDetail* send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"get_tModelDetail"

class uddi2__get_USCOREtModelDetail
{ public:
/// Vector of uddi2__tModelKey with length 0..unbounded
    std::vector<uddi2__tModelKey       > tModelKey                      1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREtModelDetail(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREtModelDetail(struct soap*, const char *tModelKey);
/// Member declared in uddi2-typemap.dat
    uddi2__get_USCOREtModelDetail(struct soap*, std::vector<char*> tModelKeys);
/// Member declared in uddi2-typemap.dat
    uddi2__tModelDetail* send(const char *endpoint);
};

/// Schema urn:uddi-org:api_v2:"save_binding"

class uddi2__save_USCOREbinding
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__bindingTemplate* with length 0..unbounded
    std::vector<uddi2__bindingTemplate*> bindingTemplate                1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREbinding(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREbinding(struct soap*, uddi2__bindingTemplate &bindingTemplate);
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREbinding(struct soap*, std::vector<uddi2__bindingTemplate*> bindingTemplates);
/// Member declared in uddi2-typemap.dat
    uddi2__bindingDetail* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"save_business"

class uddi2__save_USCOREbusiness
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__businessEntity* with length 0..unbounded
    std::vector<uddi2__businessEntity* > businessEntity                 0;
/// Vector of char* with length 0..unbounded
    std::vector<char*                  > uploadRegister                 0;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREbusiness(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREbusiness(struct soap*, uddi2__businessEntity &businessEntity);
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREbusiness(struct soap*, std::vector<uddi2__businessEntity*> businessEntities);
/// Member declared in uddi2-typemap.dat
    uddi2__businessDetail* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"save_service"

class uddi2__save_USCOREservice
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__businessService* with length 0..unbounded
    std::vector<uddi2__businessService*> businessService                1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREservice(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREservice(struct soap*, uddi2__businessService &businessService);
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREservice(struct soap*, std::vector<uddi2__businessService*> businessServices);
/// Member declared in uddi2-typemap.dat
    uddi2__serviceDetail* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"save_tModel"

class uddi2__save_USCOREtModel
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__tModel* with length 0..unbounded
    std::vector<uddi2__tModel*         > tModel                         0;
/// Vector of char* with length 0..unbounded
    std::vector<char*                  > uploadRegister                 0;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREtModel(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREtModel(struct soap*, uddi2__tModel &tModel);
/// Member declared in uddi2-typemap.dat
    uddi2__save_USCOREtModel(struct soap*, std::vector<uddi2__tModel*> tModels);
/// Member declared in uddi2-typemap.dat
    uddi2__tModelDetail* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"set_publisherAssertions"

class uddi2__set_USCOREpublisherAssertions
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Vector of uddi2__publisherAssertion* with length 0..unbounded
    std::vector<uddi2__publisherAssertion*> publisherAssertion             0;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
/// Member declared in uddi2-typemap.dat
    uddi2__set_USCOREpublisherAssertions(struct soap*);
/// Member declared in uddi2-typemap.dat
    uddi2__set_USCOREpublisherAssertions(struct soap*, std::vector<uddi2__publisherAssertion*> publisherAssertions);
/// Member declared in uddi2-typemap.dat
    uddi2__publisherAssertions* send(const char *endpoint, char *authInfo);
};

/// Schema urn:uddi-org:api_v2:"validate_values"

class uddi2__validate_USCOREvalues
{ public:
// CHOICE OF ELEMENTS:
/// Vector of uddi2__businessEntity* with length 0..unbounded
    std::vector<uddi2__businessEntity* > businessEntity                 0;
/// Vector of uddi2__businessService* with length 0..unbounded
    std::vector<uddi2__businessService*> businessService                0;
/// Vector of uddi2__tModel* with length 0..unbounded
    std::vector<uddi2__tModel*         > tModel                         0;
// END CHOICE
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"assertionStatusReport"

class uddi2__assertionStatusReport
{ public:
/// Vector of uddi2__assertionStatusItem* with length 0..unbounded
    std::vector<uddi2__assertionStatusItem*> assertionStatusItem            0;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"authToken"

class uddi2__authToken
{ public:
/// Element reference "urn:uddi-org:api_v2":authInfo
    char*                                authInfo                       1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"bindingDetail"

class uddi2__bindingDetail
{ public:
/// Vector of uddi2__bindingTemplate* with length 0..unbounded
    std::vector<uddi2__bindingTemplate*> bindingTemplate                0;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"businessDetail"

class uddi2__businessDetail
{ public:
/// Vector of uddi2__businessEntity* with length 0..unbounded
    std::vector<uddi2__businessEntity* > businessEntity                 0;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"businessDetailExt"

class uddi2__businessDetailExt
{ public:
/// Vector of uddi2__businessEntityExt* with length 0..unbounded
    std::vector<uddi2__businessEntityExt*> businessEntityExt              1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"businessList"

class uddi2__businessList
{ public:
/// Element reference "urn:uddi-org:api_v2":businessInfos
    uddi2__businessInfos*                businessInfos                  1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"publisherAssertions"

class uddi2__publisherAssertions
{ public:
/// Vector of uddi2__publisherAssertion* with length 0..unbounded
    std::vector<uddi2__publisherAssertion*> publisherAssertion             0;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute authorizedName of type xs:string
   @char*                                authorizedName                 1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"registeredInfo"

class uddi2__registeredInfo
{ public:
/// Element reference "urn:uddi-org:api_v2":businessInfos
    uddi2__businessInfos*                businessInfos                  1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":tModelInfos
    uddi2__tModelInfos*                  tModelInfos                    1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"relatedBusinessesList"

class uddi2__relatedBusinessesList
{ public:
/// Element reference "urn:uddi-org:api_v2":businessKey
    uddi2__businessKey                   businessKey                    1;	///< Required element
/// Element reference "urn:uddi-org:api_v2":relatedBusinessInfos
    uddi2__relatedBusinessInfos*         relatedBusinessInfos           1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"serviceDetail"

class uddi2__serviceDetail
{ public:
/// Vector of uddi2__businessService* with length 0..unbounded
    std::vector<uddi2__businessService*> businessService                0;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"serviceList"

class uddi2__serviceList
{ public:
/// Element reference "urn:uddi-org:api_v2":serviceInfos
    uddi2__serviceInfos*                 serviceInfos                   1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"tModelDetail"

class uddi2__tModelDetail
{ public:
/// Vector of uddi2__tModel* with length 0..unbounded
    std::vector<uddi2__tModel*         > tModel                         1;
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"tModelList"

class uddi2__tModelList
{ public:
/// Element reference "urn:uddi-org:api_v2":tModelInfos
    uddi2__tModelInfos*                  tModelInfos                    1;	///< Required element
/// Attribute generic of type xs:string
   @char*                                generic                        1;	///< Required attribute
/// Attribute operator of type xs:string
   @char*                                operator_                      1;	///< Required attribute
/// Attribute truncated of type "urn:uddi-org:api_v2":truncated
   @enum uddi2__truncated*               truncated                      0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"accessPoint"

class uddi2__accessPoint
{ public:
    char*                                __item                        ;
/// Attribute URLType of type "urn:uddi-org:api_v2":URLType
   @enum uddi2__URLType                  URLType                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"addressLine"

class uddi2__addressLine
{ public:
    char*                                __item                        ;
/// Attribute keyName of type xs:string
   @char*                                keyName                        0;	///< Optional attribute
/// Attribute keyValue of type xs:string
   @char*                                keyValue                       0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"description"

class uddi2__description
{ public:
    char*                                __item                        ;
/// Attribute reference xml:lang
   @xml__lang                            xml__lang_                     0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"discoveryURL"

class uddi2__discoveryURL
{ public:
    char*                                __item                        ;
/// Attribute useType of type xs:string
   @char*                                useType                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"email"

class uddi2__email
{ public:
    char*                                __item                        ;
/// Attribute useType of type xs:string
   @char*                                useType                        0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"errInfo"

class uddi2__errInfo
{ public:
    char*                                __item                        ;
/// Attribute errCode of type xs:string
   @char*                                errCode                        1;	///< Required attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"name"

class uddi2__name
{ public:
    char*                                __item                        ;
/// Attribute reference xml:lang
   @xml__lang                            xml__lang_                     0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/// Schema urn:uddi-org:api_v2:"phone"

class uddi2__phone
{ public:
    char*                                __item                        ;
/// Attribute useType of type xs:string
   @char*                                useType                        0;	///< Optional attribute
/// A handle to the soap struct context that manages this class instance
    struct soap                         *soap                          ;
};

/******************************************************************************\
 *                                                                            *
 * Services                                                                   *
 *                                                                            *
\******************************************************************************/

//gsoap inq2 service name:	InquireSoap 
//gsoap inq2 service type:	Inquire 
//gsoap inq2 service namespace:	urn:uddi-org:inquiry_v2 
//gsoap pub2 service name:	PublishSoap 
//gsoap pub2 service type:	Publish 
//gsoap pub2 service namespace:	urn:uddi-org:publication_v2 

/** @mainpage Service Definitions

@section Service_bindings Bindings
  - @ref InquireSoap
  - @ref PublishSoap

*/

/**

@page InquireSoap Binding "InquireSoap"

@section InquireSoap_operations Operations of Binding  "InquireSoap"
  - @ref __inq2__find_USCOREbinding
  - @ref __inq2__find_USCOREbusiness
  - @ref __inq2__find_USCORErelatedBusinesses
  - @ref __inq2__find_USCOREservice
  - @ref __inq2__find_USCOREtModel
  - @ref __inq2__get_USCOREbindingDetail
  - @ref __inq2__get_USCOREbusinessDetail
  - @ref __inq2__get_USCOREbusinessDetailExt
  - @ref __inq2__get_USCOREserviceDetail
  - @ref __inq2__get_USCOREtModelDetail

@section InquireSoap_ports Endpoints of Binding  "InquireSoap"

*/

/**

@page PublishSoap Binding "PublishSoap"

@section PublishSoap_operations Operations of Binding  "PublishSoap"
  - @ref __pub2__add_USCOREpublisherAssertions
  - @ref __pub2__delete_USCOREbinding
  - @ref __pub2__delete_USCOREbusiness
  - @ref __pub2__delete_USCOREpublisherAssertions
  - @ref __pub2__delete_USCOREservice
  - @ref __pub2__delete_USCOREtModel
  - @ref __pub2__discard_USCOREauthToken
  - @ref __pub2__get_USCOREassertionStatusReport
  - @ref __pub2__get_USCOREauthToken
  - @ref __pub2__get_USCOREpublisherAssertions
  - @ref __pub2__get_USCOREregisteredInfo
  - @ref __pub2__save_USCOREbinding
  - @ref __pub2__save_USCOREbusiness
  - @ref __pub2__save_USCOREservice
  - @ref __pub2__save_USCOREtModel
  - @ref __pub2__set_USCOREpublisherAssertions

@section PublishSoap_ports Endpoints of Binding  "PublishSoap"

*/

/******************************************************************************\
 *                                                                            *
 * SOAP Fault Detail                                                          *
 *                                                                            *
\******************************************************************************/

/// The SOAP Fault Detail element contains one of the following types serialized
// in the __type and fault fields of the SOAP_ENV__Detail struct (see docs)
class _dispositionReport
{ public:
    uddi2__dispositionReport*           uddi2__dispositionReport_;
    struct soap                         *soap                          ;
};

/******************************************************************************\
 *                                                                            *
 * InquireSoap                                                                *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * __inq2__find_USCOREbinding                                                 *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__find_USCOREbinding" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="find_binding"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__find_USCOREbinding(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__find_USCOREbinding*          uddi2__find_USCOREbinding_,
    // response parameters:
    uddi2__bindingDetail*               uddi2__bindingDetail_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	find_USCOREbinding document
//gsoap inq2 service method-encoding:	find_USCOREbinding literal
//gsoap inq2 service method-action:	find_USCOREbinding find_binding
//gsoap inq2 service method-fault:	find_USCOREbinding _dispositionReport
int __inq2__find_USCOREbinding(
    uddi2__find_USCOREbinding*          uddi2__find_USCOREbinding_,
    uddi2__bindingDetail*               uddi2__bindingDetail_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __inq2__find_USCOREbusiness                                                *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__find_USCOREbusiness" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="find_business"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__find_USCOREbusiness(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__find_USCOREbusiness*         uddi2__find_USCOREbusiness_,
    // response parameters:
    uddi2__businessList*                uddi2__businessList_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	find_USCOREbusiness document
//gsoap inq2 service method-encoding:	find_USCOREbusiness literal
//gsoap inq2 service method-action:	find_USCOREbusiness find_business
//gsoap inq2 service method-fault:	find_USCOREbusiness _dispositionReport
int __inq2__find_USCOREbusiness(
    uddi2__find_USCOREbusiness*         uddi2__find_USCOREbusiness_,
    uddi2__businessList*                uddi2__businessList_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __inq2__find_USCORErelatedBusinesses                                       *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__find_USCORErelatedBusinesses" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="find_relatedBusinesses"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__find_USCORErelatedBusinesses(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__find_USCORErelatedBusinesses* uddi2__find_USCORErelatedBusinesses_,
    // response parameters:
    uddi2__relatedBusinessesList*       uddi2__relatedBusinessesList_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	find_USCORErelatedBusinesses document
//gsoap inq2 service method-encoding:	find_USCORErelatedBusinesses literal
//gsoap inq2 service method-action:	find_USCORErelatedBusinesses find_relatedBusinesses
//gsoap inq2 service method-fault:	find_USCORErelatedBusinesses _dispositionReport
int __inq2__find_USCORErelatedBusinesses(
    uddi2__find_USCORErelatedBusinesses* uddi2__find_USCORErelatedBusinesses_,
    uddi2__relatedBusinessesList*       uddi2__relatedBusinessesList_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __inq2__find_USCOREservice                                                 *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__find_USCOREservice" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="find_service"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__find_USCOREservice(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__find_USCOREservice*          uddi2__find_USCOREservice_,
    // response parameters:
    uddi2__serviceList*                 uddi2__serviceList_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	find_USCOREservice document
//gsoap inq2 service method-encoding:	find_USCOREservice literal
//gsoap inq2 service method-action:	find_USCOREservice find_service
//gsoap inq2 service method-fault:	find_USCOREservice _dispositionReport
int __inq2__find_USCOREservice(
    uddi2__find_USCOREservice*          uddi2__find_USCOREservice_,
    uddi2__serviceList*                 uddi2__serviceList_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __inq2__find_USCOREtModel                                                  *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__find_USCOREtModel" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="find_tModel"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__find_USCOREtModel(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__find_USCOREtModel*           uddi2__find_USCOREtModel_,
    // response parameters:
    uddi2__tModelList*                  uddi2__tModelList_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	find_USCOREtModel document
//gsoap inq2 service method-encoding:	find_USCOREtModel literal
//gsoap inq2 service method-action:	find_USCOREtModel find_tModel
//gsoap inq2 service method-fault:	find_USCOREtModel _dispositionReport
int __inq2__find_USCOREtModel(
    uddi2__find_USCOREtModel*           uddi2__find_USCOREtModel_,
    uddi2__tModelList*                  uddi2__tModelList_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __inq2__get_USCOREbindingDetail                                            *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__get_USCOREbindingDetail" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="get_bindingDetail"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__get_USCOREbindingDetail(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__get_USCOREbindingDetail*     uddi2__get_USCOREbindingDetail_,
    // response parameters:
    uddi2__bindingDetail*               uddi2__bindingDetail_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	get_USCOREbindingDetail document
//gsoap inq2 service method-encoding:	get_USCOREbindingDetail literal
//gsoap inq2 service method-action:	get_USCOREbindingDetail get_bindingDetail
//gsoap inq2 service method-fault:	get_USCOREbindingDetail _dispositionReport
int __inq2__get_USCOREbindingDetail(
    uddi2__get_USCOREbindingDetail*     uddi2__get_USCOREbindingDetail_,
    uddi2__bindingDetail*               uddi2__bindingDetail_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __inq2__get_USCOREbusinessDetail                                           *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__get_USCOREbusinessDetail" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="get_businessDetail"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__get_USCOREbusinessDetail(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__get_USCOREbusinessDetail*    uddi2__get_USCOREbusinessDetail_,
    // response parameters:
    uddi2__businessDetail*              uddi2__businessDetail_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	get_USCOREbusinessDetail document
//gsoap inq2 service method-encoding:	get_USCOREbusinessDetail literal
//gsoap inq2 service method-action:	get_USCOREbusinessDetail get_businessDetail
//gsoap inq2 service method-fault:	get_USCOREbusinessDetail _dispositionReport
int __inq2__get_USCOREbusinessDetail(
    uddi2__get_USCOREbusinessDetail*    uddi2__get_USCOREbusinessDetail_,
    uddi2__businessDetail*              uddi2__businessDetail_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __inq2__get_USCOREbusinessDetailExt                                        *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__get_USCOREbusinessDetailExt" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="get_businessDetailExt"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__get_USCOREbusinessDetailExt(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__get_USCOREbusinessDetailExt* uddi2__get_USCOREbusinessDetailExt_,
    // response parameters:
    uddi2__businessDetailExt*           uddi2__businessDetailExt_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	get_USCOREbusinessDetailExt document
//gsoap inq2 service method-encoding:	get_USCOREbusinessDetailExt literal
//gsoap inq2 service method-action:	get_USCOREbusinessDetailExt get_businessDetailExt
//gsoap inq2 service method-fault:	get_USCOREbusinessDetailExt _dispositionReport
int __inq2__get_USCOREbusinessDetailExt(
    uddi2__get_USCOREbusinessDetailExt* uddi2__get_USCOREbusinessDetailExt_,
    uddi2__businessDetailExt*           uddi2__businessDetailExt_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __inq2__get_USCOREserviceDetail                                            *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__get_USCOREserviceDetail" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="get_serviceDetail"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__get_USCOREserviceDetail(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__get_USCOREserviceDetail*     uddi2__get_USCOREserviceDetail_,
    // response parameters:
    uddi2__serviceDetail*               uddi2__serviceDetail_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	get_USCOREserviceDetail document
//gsoap inq2 service method-encoding:	get_USCOREserviceDetail literal
//gsoap inq2 service method-action:	get_USCOREserviceDetail get_serviceDetail
//gsoap inq2 service method-fault:	get_USCOREserviceDetail _dispositionReport
int __inq2__get_USCOREserviceDetail(
    uddi2__get_USCOREserviceDetail*     uddi2__get_USCOREserviceDetail_,
    uddi2__serviceDetail*               uddi2__serviceDetail_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __inq2__get_USCOREtModelDetail                                             *
 *                                                                            *
\******************************************************************************/


/// Operation "__inq2__get_USCOREtModelDetail" of service binding "InquireSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="get_tModelDetail"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___inq2__get_USCOREtModelDetail(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__get_USCOREtModelDetail*      uddi2__get_USCOREtModelDetail_,
    // response parameters:
    uddi2__tModelDetail*                uddi2__tModelDetail_
  );
@endcode

C++ proxy class (defined in soapInquireSoapProxy.h):
  class InquireSoap;

*/

//gsoap inq2 service method-style:	get_USCOREtModelDetail document
//gsoap inq2 service method-encoding:	get_USCOREtModelDetail literal
//gsoap inq2 service method-action:	get_USCOREtModelDetail get_tModelDetail
//gsoap inq2 service method-fault:	get_USCOREtModelDetail _dispositionReport
int __inq2__get_USCOREtModelDetail(
    uddi2__get_USCOREtModelDetail*      uddi2__get_USCOREtModelDetail_,
    uddi2__tModelDetail*                uddi2__tModelDetail_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * PublishSoap                                                                *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * __pub2__add_USCOREpublisherAssertions                                      *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__add_USCOREpublisherAssertions" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="add_publisherAssertions"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__add_USCOREpublisherAssertions(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__add_USCOREpublisherAssertions* uddi2__add_USCOREpublisherAssertions_,
    // response parameters:
    uddi2__dispositionReport*           uddi2__dispositionReport_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	add_USCOREpublisherAssertions document
//gsoap pub2 service method-encoding:	add_USCOREpublisherAssertions literal
//gsoap pub2 service method-action:	add_USCOREpublisherAssertions add_publisherAssertions
//gsoap pub2 service method-fault:	add_USCOREpublisherAssertions _dispositionReport
int __pub2__add_USCOREpublisherAssertions(
    uddi2__add_USCOREpublisherAssertions* uddi2__add_USCOREpublisherAssertions_,
    uddi2__dispositionReport*           uddi2__dispositionReport_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__delete_USCOREbinding                                               *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__delete_USCOREbinding" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="delete_binding"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__delete_USCOREbinding(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__delete_USCOREbinding*        uddi2__delete_USCOREbinding_,
    // response parameters:
    uddi2__dispositionReport*           uddi2__dispositionReport_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	delete_USCOREbinding document
//gsoap pub2 service method-encoding:	delete_USCOREbinding literal
//gsoap pub2 service method-action:	delete_USCOREbinding delete_binding
//gsoap pub2 service method-fault:	delete_USCOREbinding _dispositionReport
int __pub2__delete_USCOREbinding(
    uddi2__delete_USCOREbinding*        uddi2__delete_USCOREbinding_,
    uddi2__dispositionReport*           uddi2__dispositionReport_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__delete_USCOREbusiness                                              *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__delete_USCOREbusiness" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="delete_business"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__delete_USCOREbusiness(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__delete_USCOREbusiness*       uddi2__delete_USCOREbusiness_,
    // response parameters:
    uddi2__dispositionReport*           uddi2__dispositionReport_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	delete_USCOREbusiness document
//gsoap pub2 service method-encoding:	delete_USCOREbusiness literal
//gsoap pub2 service method-action:	delete_USCOREbusiness delete_business
//gsoap pub2 service method-fault:	delete_USCOREbusiness _dispositionReport
int __pub2__delete_USCOREbusiness(
    uddi2__delete_USCOREbusiness*       uddi2__delete_USCOREbusiness_,
    uddi2__dispositionReport*           uddi2__dispositionReport_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__delete_USCOREpublisherAssertions                                   *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__delete_USCOREpublisherAssertions" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="delete_publisherAssertions"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__delete_USCOREpublisherAssertions(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__delete_USCOREpublisherAssertions* uddi2__delete_USCOREpublisherAssertions_,
    // response parameters:
    uddi2__dispositionReport*           uddi2__dispositionReport_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	delete_USCOREpublisherAssertions document
//gsoap pub2 service method-encoding:	delete_USCOREpublisherAssertions literal
//gsoap pub2 service method-action:	delete_USCOREpublisherAssertions delete_publisherAssertions
//gsoap pub2 service method-fault:	delete_USCOREpublisherAssertions _dispositionReport
int __pub2__delete_USCOREpublisherAssertions(
    uddi2__delete_USCOREpublisherAssertions* uddi2__delete_USCOREpublisherAssertions_,
    uddi2__dispositionReport*           uddi2__dispositionReport_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__delete_USCOREservice                                               *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__delete_USCOREservice" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="delete_service"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__delete_USCOREservice(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__delete_USCOREservice*        uddi2__delete_USCOREservice_,
    // response parameters:
    uddi2__dispositionReport*           uddi2__dispositionReport_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	delete_USCOREservice document
//gsoap pub2 service method-encoding:	delete_USCOREservice literal
//gsoap pub2 service method-action:	delete_USCOREservice delete_service
//gsoap pub2 service method-fault:	delete_USCOREservice _dispositionReport
int __pub2__delete_USCOREservice(
    uddi2__delete_USCOREservice*        uddi2__delete_USCOREservice_,
    uddi2__dispositionReport*           uddi2__dispositionReport_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__delete_USCOREtModel                                                *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__delete_USCOREtModel" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="delete_tModel"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__delete_USCOREtModel(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__delete_USCOREtModel*         uddi2__delete_USCOREtModel_,
    // response parameters:
    uddi2__dispositionReport*           uddi2__dispositionReport_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	delete_USCOREtModel document
//gsoap pub2 service method-encoding:	delete_USCOREtModel literal
//gsoap pub2 service method-action:	delete_USCOREtModel delete_tModel
//gsoap pub2 service method-fault:	delete_USCOREtModel _dispositionReport
int __pub2__delete_USCOREtModel(
    uddi2__delete_USCOREtModel*         uddi2__delete_USCOREtModel_,
    uddi2__dispositionReport*           uddi2__dispositionReport_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__discard_USCOREauthToken                                            *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__discard_USCOREauthToken" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="discard_authToken"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__discard_USCOREauthToken(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__discard_USCOREauthToken*     uddi2__discard_USCOREauthToken_,
    // response parameters:
    uddi2__dispositionReport*           uddi2__dispositionReport_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	discard_USCOREauthToken document
//gsoap pub2 service method-encoding:	discard_USCOREauthToken literal
//gsoap pub2 service method-action:	discard_USCOREauthToken discard_authToken
//gsoap pub2 service method-fault:	discard_USCOREauthToken _dispositionReport
int __pub2__discard_USCOREauthToken(
    uddi2__discard_USCOREauthToken*     uddi2__discard_USCOREauthToken_,
    uddi2__dispositionReport*           uddi2__dispositionReport_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__get_USCOREassertionStatusReport                                    *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__get_USCOREassertionStatusReport" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="get_assertionStatusReport"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__get_USCOREassertionStatusReport(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__get_USCOREassertionStatusReport* uddi2__get_USCOREassertionStatusReport_,
    // response parameters:
    uddi2__assertionStatusReport*       uddi2__assertionStatusReport_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	get_USCOREassertionStatusReport document
//gsoap pub2 service method-encoding:	get_USCOREassertionStatusReport literal
//gsoap pub2 service method-action:	get_USCOREassertionStatusReport get_assertionStatusReport
//gsoap pub2 service method-fault:	get_USCOREassertionStatusReport _dispositionReport
int __pub2__get_USCOREassertionStatusReport(
    uddi2__get_USCOREassertionStatusReport* uddi2__get_USCOREassertionStatusReport_,
    uddi2__assertionStatusReport*       uddi2__assertionStatusReport_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__get_USCOREauthToken                                                *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__get_USCOREauthToken" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="get_authToken"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__get_USCOREauthToken(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__get_USCOREauthToken*         uddi2__get_USCOREauthToken_,
    // response parameters:
    uddi2__authToken*                   uddi2__authToken_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	get_USCOREauthToken document
//gsoap pub2 service method-encoding:	get_USCOREauthToken literal
//gsoap pub2 service method-action:	get_USCOREauthToken get_authToken
//gsoap pub2 service method-fault:	get_USCOREauthToken _dispositionReport
int __pub2__get_USCOREauthToken(
    uddi2__get_USCOREauthToken*         uddi2__get_USCOREauthToken_,
    uddi2__authToken*                   uddi2__authToken_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__get_USCOREpublisherAssertions                                      *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__get_USCOREpublisherAssertions" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="get_publisherAssertions"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__get_USCOREpublisherAssertions(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__get_USCOREpublisherAssertions* uddi2__get_USCOREpublisherAssertions_,
    // response parameters:
    uddi2__publisherAssertions*         uddi2__publisherAssertions_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	get_USCOREpublisherAssertions document
//gsoap pub2 service method-encoding:	get_USCOREpublisherAssertions literal
//gsoap pub2 service method-action:	get_USCOREpublisherAssertions get_publisherAssertions
//gsoap pub2 service method-fault:	get_USCOREpublisherAssertions _dispositionReport
int __pub2__get_USCOREpublisherAssertions(
    uddi2__get_USCOREpublisherAssertions* uddi2__get_USCOREpublisherAssertions_,
    uddi2__publisherAssertions*         uddi2__publisherAssertions_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__get_USCOREregisteredInfo                                           *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__get_USCOREregisteredInfo" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="get_registeredInfo"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__get_USCOREregisteredInfo(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__get_USCOREregisteredInfo*    uddi2__get_USCOREregisteredInfo_,
    // response parameters:
    uddi2__registeredInfo*              uddi2__registeredInfo_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	get_USCOREregisteredInfo document
//gsoap pub2 service method-encoding:	get_USCOREregisteredInfo literal
//gsoap pub2 service method-action:	get_USCOREregisteredInfo get_registeredInfo
//gsoap pub2 service method-fault:	get_USCOREregisteredInfo _dispositionReport
int __pub2__get_USCOREregisteredInfo(
    uddi2__get_USCOREregisteredInfo*    uddi2__get_USCOREregisteredInfo_,
    uddi2__registeredInfo*              uddi2__registeredInfo_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__save_USCOREbinding                                                 *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__save_USCOREbinding" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="save_binding"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__save_USCOREbinding(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__save_USCOREbinding*          uddi2__save_USCOREbinding_,
    // response parameters:
    uddi2__bindingDetail*               uddi2__bindingDetail_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	save_USCOREbinding document
//gsoap pub2 service method-encoding:	save_USCOREbinding literal
//gsoap pub2 service method-action:	save_USCOREbinding save_binding
//gsoap pub2 service method-fault:	save_USCOREbinding _dispositionReport
int __pub2__save_USCOREbinding(
    uddi2__save_USCOREbinding*          uddi2__save_USCOREbinding_,
    uddi2__bindingDetail*               uddi2__bindingDetail_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__save_USCOREbusiness                                                *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__save_USCOREbusiness" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="save_business"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__save_USCOREbusiness(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__save_USCOREbusiness*         uddi2__save_USCOREbusiness_,
    // response parameters:
    uddi2__businessDetail*              uddi2__businessDetail_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	save_USCOREbusiness document
//gsoap pub2 service method-encoding:	save_USCOREbusiness literal
//gsoap pub2 service method-action:	save_USCOREbusiness save_business
//gsoap pub2 service method-fault:	save_USCOREbusiness _dispositionReport
int __pub2__save_USCOREbusiness(
    uddi2__save_USCOREbusiness*         uddi2__save_USCOREbusiness_,
    uddi2__businessDetail*              uddi2__businessDetail_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__save_USCOREservice                                                 *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__save_USCOREservice" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="save_service"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__save_USCOREservice(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__save_USCOREservice*          uddi2__save_USCOREservice_,
    // response parameters:
    uddi2__serviceDetail*               uddi2__serviceDetail_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	save_USCOREservice document
//gsoap pub2 service method-encoding:	save_USCOREservice literal
//gsoap pub2 service method-action:	save_USCOREservice save_service
//gsoap pub2 service method-fault:	save_USCOREservice _dispositionReport
int __pub2__save_USCOREservice(
    uddi2__save_USCOREservice*          uddi2__save_USCOREservice_,
    uddi2__serviceDetail*               uddi2__serviceDetail_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__save_USCOREtModel                                                  *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__save_USCOREtModel" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="save_tModel"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__save_USCOREtModel(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__save_USCOREtModel*           uddi2__save_USCOREtModel_,
    // response parameters:
    uddi2__tModelDetail*                uddi2__tModelDetail_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	save_USCOREtModel document
//gsoap pub2 service method-encoding:	save_USCOREtModel literal
//gsoap pub2 service method-action:	save_USCOREtModel save_tModel
//gsoap pub2 service method-fault:	save_USCOREtModel _dispositionReport
int __pub2__save_USCOREtModel(
    uddi2__save_USCOREtModel*           uddi2__save_USCOREtModel_,
    uddi2__tModelDetail*                uddi2__tModelDetail_ ///< response parameter
);

/******************************************************************************\
 *                                                                            *
 * __pub2__set_USCOREpublisherAssertions                                      *
 *                                                                            *
\******************************************************************************/


/// Operation "__pub2__set_USCOREpublisherAssertions" of service binding "PublishSoap"

/**

Operation details:

  - SOAP document/literal style
  - SOAP action="set_publisherAssertions"
  - SOAP Fault: _dispositionReport

C stub function (defined in soapClient.c[pp]):
@code
  int soap_call___pub2__set_USCOREpublisherAssertions(struct soap *soap,
    NULL, // char *endpoint = NULL selects default endpoint for this operation
    NULL, // char *action = NULL selects default action for this operation
    // request parameters:
    uddi2__set_USCOREpublisherAssertions* uddi2__set_USCOREpublisherAssertions_,
    // response parameters:
    uddi2__publisherAssertions*         uddi2__publisherAssertions_
  );
@endcode

C++ proxy class (defined in soapPublishSoapProxy.h):
  class PublishSoap;

*/

//gsoap pub2 service method-style:	set_USCOREpublisherAssertions document
//gsoap pub2 service method-encoding:	set_USCOREpublisherAssertions literal
//gsoap pub2 service method-action:	set_USCOREpublisherAssertions set_publisherAssertions
//gsoap pub2 service method-fault:	set_USCOREpublisherAssertions _dispositionReport
int __pub2__set_USCOREpublisherAssertions(
    uddi2__set_USCOREpublisherAssertions* uddi2__set_USCOREpublisherAssertions_,
    uddi2__publisherAssertions*         uddi2__publisherAssertions_ ///< response parameter
);

/* End of uddi_v2.h */
