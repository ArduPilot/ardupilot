/* Copyright 2014 Adam Green (http://mbed.org/users/AdamGreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* 'Class' used to parse and tokenize a string based on provided list of separators. */
#include <string.h>
#include "token.h"

#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))


void Token_Init(Token* pToken)
{
    Token_InitWith(pToken, " \t");
}


static void clearTokenObject(Token* pToken);
void Token_InitWith(Token* pToken, const char* pTheseTokenSeparators)
{
    clearTokenObject(pToken);
    pToken->pTokenSeparators = pTheseTokenSeparators;
}

static void clearTokenObject(Token* pToken)
{
    memset(pToken->tokenPointers, 0, sizeof(pToken->tokenPointers));
    pToken->tokenCount = 0;
    pToken->copyOfString[0] = '\0';
}


static void copyStringIntoToken(Token* pToken, const char* pStringToCopy);
static void splitStringCopyIntoTokens(Token* pToken);
static char* findFirstNonSeparator(Token* pToken, char* p);
static char* findFirstSeparator(Token* pToken, char* p);
static int charIsSeparator(Token* pToken, char c);
static void addToken(Token* pToken, const char* p);
void Token_SplitString(Token* pToken, const char* pStringToSplit)
{
    __try
    {
        clearTokenObject(pToken);
        __throwing_func( copyStringIntoToken(pToken, pStringToSplit) );
        __throwing_func( splitStringCopyIntoTokens(pToken) );
    }
    __catch
    {
        __rethrow;
    }
}

static void copyStringIntoToken(Token* pToken, const char* pStringToCopy)
{
    size_t      bytesLeft = sizeof(pToken->copyOfString);
    const char* pSource = pStringToCopy;
    char*       pDest = pToken->copyOfString;
    
    while (bytesLeft > 1 && *pSource)
    {
        *pDest++ = *pSource++;
        bytesLeft--;
    }
    *pDest = '\0';
    
    if (*pSource)
        __throw(bufferOverrunException);
}

static void splitStringCopyIntoTokens(Token* pToken)
{
    char* p = pToken->copyOfString;
    while (*p)
    {
        p = findFirstNonSeparator(pToken, p);
        __try
            addToken(pToken, p);
        __catch
            __rethrow;
        p = findFirstSeparator(pToken, p);
        if (*p)
            *p++ = '\0';
    }
}

static char* findFirstNonSeparator(Token* pToken, char* p)
{
    while (*p && charIsSeparator(pToken, *p))
        p++;
    
    return p;
}

static char* findFirstSeparator(Token* pToken, char* p)
{
    while (*p && !charIsSeparator(pToken, *p))
        p++;
    
    return p;
}

static int charIsSeparator(Token* pToken, char c)
{
    const char* pSeparator = pToken->pTokenSeparators;
    
    while (*pSeparator)
    {
        if (c == *pSeparator)
            return 1;
        pSeparator++;
    }
    
    return 0;
}

static void addToken(Token* pToken, const char* p)
{
    if ('\0' == *p)
        return;
        
    if (pToken->tokenCount >= ARRAY_SIZE(pToken->tokenPointers))
        __throw(bufferOverrunException);
        
    pToken->tokenPointers[pToken->tokenCount++] = p;
}


size_t Token_GetTokenCount(Token* pToken)
{
    return pToken->tokenCount;
}


const char* Token_GetToken(Token* pToken, size_t tokenIndex)
{
    if (tokenIndex >= pToken->tokenCount)
        __throw_and_return(invalidIndexException, NULL);

    return pToken->tokenPointers[tokenIndex];
}


const char* Token_MatchingString(Token* pToken, const char* pTokenToSearchFor)
{
    size_t i;
    
    for (i = 0 ; i < pToken->tokenCount ; i++)
    {
        if (0 == strcmp(pToken->tokenPointers[i], pTokenToSearchFor))
            return pToken->tokenPointers[i];
    }
    return NULL;
}


const char* Token_MatchingStringPrefix(Token* pToken, const char* pTokenPrefixToSearchFor)
{
    size_t i;
    
    for (i = 0 ; i < pToken->tokenCount ; i++)
    {
        if (pToken->tokenPointers[i] == strstr(pToken->tokenPointers[i], pTokenPrefixToSearchFor))
            return pToken->tokenPointers[i];
    }
    return NULL;
}


static void adjustTokenPointers(Token* pToken, const char* pOriginalStringCopyBaseAddress);
void Token_Copy(Token* pTokenCopy, Token* pTokenOriginal)
{
    *pTokenCopy = *pTokenOriginal;
    
    adjustTokenPointers(pTokenCopy, pTokenOriginal->copyOfString);
}

static void adjustTokenPointers(Token* pToken, const char* pOriginalStringCopyBaseAddress)
{
    size_t i;
    
    for (i = 0 ; i < pToken->tokenCount ; i++)
    {
        int tokenOffset = pToken->tokenPointers[i] - pOriginalStringCopyBaseAddress;
        
        pToken->tokenPointers[i] = pToken->copyOfString + tokenOffset;
    }
}
