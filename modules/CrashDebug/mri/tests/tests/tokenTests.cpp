/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

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
#include <string.h>

extern "C"
{
#include "token.h"
}

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"

#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))


TEST_GROUP(Token)
{
    Token   m_token;
    int     m_expectedExceptionCode;
    char*   m_pTestString;
    
    void setup()
    {
        Token_Init(&m_token);
        clearExceptionCode();
        m_expectedExceptionCode = noException;
        m_pTestString = NULL;
    }

    void teardown()
    {
        LONGS_EQUAL( m_expectedExceptionCode, getExceptionCode() );
        clearExceptionCode();
        free(m_pTestString);
    }
    
    void validateInitToken()
    {
        validateNullTokenPointers();
        LONGS_EQUAL( 0, m_token.tokenCount );
        LONGS_EQUAL( 0, m_token.copyOfString[0] );
    }
    
    void validateNullTokenPointers()
    {
        size_t i;
        
        for (i = 0 ; i < ARRAY_SIZE(m_token.tokenPointers) ; i++)
        {
            POINTERS_EQUAL( NULL, m_token.tokenPointers[i] );
        }
    }
    
    void validateDefaultTokenWhitspaceSeparators()
    {
        const char* defaultWhitespaceSeparators = " \t";
        STRCMP_EQUAL( defaultWhitespaceSeparators, m_token.pTokenSeparators );
    }
    
    char* allocateAndFillTestString(size_t size, char fillCharacter)
    {
        m_pTestString = (char*) malloc(size+1);
        memset(m_pTestString, fillCharacter, size);
        m_pTestString[size] = '\0';
        return m_pTestString;
    }
    
    char* allocateRepeatingTokens(size_t numberOfTokens, const char* pTokenText)
    {
        size_t tokenSize = strlen(pTokenText);
        size_t totalAllocationSize = tokenSize * numberOfTokens + 1;
        char*  p;
        
        m_pTestString = (char*)malloc(totalAllocationSize);
        p = m_pTestString;
        
        for (size_t i = 0 ; i < numberOfTokens ; i++)
        {
            memcpy(p, pTokenText, tokenSize);
            p += tokenSize;
        }
        *p = '\0';
        
        return m_pTestString;
    }
    
    void expectExceptionToBeThrown(int expectedExceptionCode)
    {
        m_expectedExceptionCode = expectedExceptionCode;
    }
    
    void validateTokenCopy(Token* pTokenCopy)
    {
        POINTERS_EQUAL( m_token.pTokenSeparators, pTokenCopy->pTokenSeparators );
        LONGS_EQUAL( m_token.tokenCount, pTokenCopy->tokenCount );
        STRCMP_EQUAL( m_token.copyOfString, pTokenCopy->copyOfString );
        
        for (size_t i = 0 ; i < m_token.tokenCount ; i++)
        {
            STRCMP_EQUAL( m_token.tokenPointers[i], pTokenCopy->tokenPointers[i] );
            // Make sure that the token pointers were adjusted.
            CHECK( m_token.tokenPointers[i] != pTokenCopy->tokenPointers[i] );
        }
    }
};

TEST(Token, Token_Init)
{
    Token_Init(&m_token);
    validateInitToken();
    validateDefaultTokenWhitspaceSeparators();
}

TEST(Token, Token_InitWith)
{
    Token_InitWith(&m_token, ",");
    validateInitToken();
    STRCMP_EQUAL( ",", m_token.pTokenSeparators );
}

TEST(Token, Token_SplitString_NoSeparators)
{
    Token_SplitString(&m_token, "Test");
    LONGS_EQUAL( 1, Token_GetTokenCount(&m_token) );
    STRCMP_EQUAL( "Test", Token_GetToken(&m_token, 0) );
}

TEST(Token, Token_SplitString_TwoTokens)
{
    Token_SplitString(&m_token, "Test Tokens");
    LONGS_EQUAL( 2, Token_GetTokenCount(&m_token) );
    STRCMP_EQUAL( "Test", Token_GetToken(&m_token, 0) );
    STRCMP_EQUAL( "Tokens", Token_GetToken(&m_token, 1) );
}

TEST(Token, Token_SplitString_LeadingSeparator)
{
    Token_SplitString(&m_token, " Test");
    LONGS_EQUAL( 1, Token_GetTokenCount(&m_token) );
    STRCMP_EQUAL( "Test", Token_GetToken(&m_token, 0) );
}

TEST(Token, Token_SplitString_TrailingSeparator)
{
    Token_SplitString(&m_token, "Test ");
    LONGS_EQUAL( 1, Token_GetTokenCount(&m_token) );
    STRCMP_EQUAL( "Test", Token_GetToken(&m_token, 0) );
}

TEST(Token, Token_SplitString_TrailingSeparators)
{
    Token_SplitString(&m_token, "Test  ");
    LONGS_EQUAL( 1, Token_GetTokenCount(&m_token) );
    STRCMP_EQUAL( "Test", Token_GetToken(&m_token, 0) );
}

TEST(Token, Token_SplitString_MultipleSeparators)
{
    Token_SplitString(&m_token, "Test     Tokens");
    LONGS_EQUAL( 2, Token_GetTokenCount(&m_token) );
    STRCMP_EQUAL( "Test", Token_GetToken(&m_token, 0) );
    STRCMP_EQUAL( "Tokens", Token_GetToken(&m_token, 1) );
}

TEST(Token, Token_SplitString_VariousSeparators)
{
    Token_SplitString(&m_token, "Test \t\t  Tokens");
    LONGS_EQUAL( 2, Token_GetTokenCount(&m_token) );
    STRCMP_EQUAL( "Test", Token_GetToken(&m_token, 0) );
    STRCMP_EQUAL( "Tokens", Token_GetToken(&m_token, 1) );
}

TEST(Token, Token_SplitString_MaximumString)
{
    Token_SplitString(&m_token, allocateAndFillTestString(TOKEN_MAX_STRING, '_'));
}

TEST(Token, Token_SplitString_TooLargeString)
{
    Token_SplitString(&m_token, allocateAndFillTestString(TOKEN_MAX_STRING + 1, '_'));
    expectExceptionToBeThrown(bufferOverrunException);
}

TEST(Token, Token_SplitString_MaximumTokens)
{
    Token_SplitString(&m_token, allocateRepeatingTokens(TOKEN_MAX_TOKENS, "test "));
    LONGS_EQUAL( TOKEN_MAX_TOKENS, Token_GetTokenCount(&m_token) );
}

TEST(Token, Token_SplitString_TooManyTokens)
{
    Token_SplitString(&m_token, allocateRepeatingTokens(TOKEN_MAX_TOKENS + 1, "test "));
    validateDefaultTokenWhitspaceSeparators();
    expectExceptionToBeThrown(bufferOverrunException);
}

TEST(Token, Token_GetToken_IndexOutOfRange)
{
    Token_SplitString(&m_token, "Test");
    clearExceptionCode();
    POINTERS_EQUAL( NULL, Token_GetToken(&m_token, 1) );
    expectExceptionToBeThrown(invalidIndexException);
}

TEST(Token, Token_MatchingString_FoundInFirstToken)
{
    const char* pMatchResult = NULL;
    
    Token_SplitString(&m_token, "Test Tokens");
    clearExceptionCode();
    
    pMatchResult = Token_MatchingString(&m_token, "Test");
    STRCMP_EQUAL( "Test", pMatchResult );
}

TEST(Token, Token_MatchingString_FoundInSecondToken)
{
    const char* pMatchResult = NULL;
    
    Token_SplitString(&m_token, "Test Tokens");
    clearExceptionCode();
    
    pMatchResult = Token_MatchingString(&m_token, "Tokens");
    STRCMP_EQUAL( "Tokens", pMatchResult );
}

TEST(Token, Token_MatchingString_NotFound)
{
    const char* pMatchResult = NULL;
    
    Token_SplitString(&m_token, "Test Tokens");
    clearExceptionCode();
    
    pMatchResult = Token_MatchingString(&m_token, "NotFound");
    POINTERS_EQUAL( NULL, pMatchResult );
}

TEST(Token, Token_MatchingString_NotFoundBecauseOfCaseMismatch)
{
    const char* pMatchResult = NULL;
    
    Token_SplitString(&m_token, "Test Tokens");
    clearExceptionCode();
    
    pMatchResult = Token_MatchingString(&m_token, "TesT");
    POINTERS_EQUAL( NULL, pMatchResult );
}

TEST(Token, Token_MatchingStringPrefix_FoundInFirstToken)
{
    const char* pMatchResult = NULL;
    
    Token_SplitString(&m_token, "Test=Value1 Tokens=Value2");
    clearExceptionCode();
    
    pMatchResult = Token_MatchingStringPrefix(&m_token, "Test=");
    STRCMP_EQUAL( "Test=Value1", pMatchResult );
}

TEST(Token, Token_MatchingStringPrefix_FoundInSecondToken)
{
    const char* pMatchResult = NULL;
    
    Token_SplitString(&m_token, "Test=Value1 Tokens=Value2");
    clearExceptionCode();
    
    pMatchResult = Token_MatchingStringPrefix(&m_token, "Tokens=");
    STRCMP_EQUAL( "Tokens=Value2", pMatchResult );
}

TEST(Token, Token_MatchingStringPrefix_NotFound)
{
    const char* pMatchResult = NULL;
    
    Token_SplitString(&m_token, "Test=Value1 Tokens=Value2");
    clearExceptionCode();
    
    pMatchResult = Token_MatchingStringPrefix(&m_token, "NotFound=");
    POINTERS_EQUAL( NULL, pMatchResult );
}

TEST(Token, Token_MatchingStringPrefix_NotFoundBecauseOfCaseMismatch)
{
    const char* pMatchResult = NULL;
    
    Token_SplitString(&m_token, "Test=Value1 Tokens=Value2");
    clearExceptionCode();
    
    pMatchResult = Token_MatchingStringPrefix(&m_token, "TesT=");
    POINTERS_EQUAL( NULL, pMatchResult );
}

TEST(Token, Token_Copy_OnEmptyObject)
{
    Token tokenCopy;
    Token_Copy(&tokenCopy, &m_token);
    validateTokenCopy(&tokenCopy);
}

TEST(Token, Token_Copy_OneToken)
{
    Token tokenCopy;
    Token_SplitString(&m_token, "Test");
    Token_Copy(&tokenCopy, &m_token);
    validateTokenCopy(&tokenCopy);
}

TEST(Token, Token_Copy_FullTokenPointerArray)
{
    Token tokenCopy;
    Token_SplitString(&m_token, allocateRepeatingTokens(TOKEN_MAX_TOKENS, "test "));
    Token_Copy(&tokenCopy, &m_token);
    validateTokenCopy(&tokenCopy);
}
