/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    paths.c
 * @brief   Path utilities code.
 *
 * @addtogroup UTILS_PATHS
 * @{
 */

#include <string.h>
#include <stdbool.h>

#include "errcodes.h"
#include "paths.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Appends a path to a path.
 *
 * @param[in, out]  dst         The destination buffer.
 * @param[in]       src         The source path.
 * @param[in]       size        Destination buffer size.
 * @return                      The size of the combined path.
 * @retval 0                    Path error or buffer overflow.
 */
size_t path_append(char *dst, const char *src, size_t size) {
  size_t n;

  /* Current path length.*/
  n = strnlen(dst, size);
  if (n >= size) {
    return (size_t)0;
  }

  /* Making sure to start with a separator in place.*/
  if (n == 0U) {
    *dst++ = '/';
    n++;
  }
  else {
    dst = dst + n;
    if (*(dst - 1) != '/') {
      *dst++ = '/';
      n++;
    }
  }

  /* The appended part needs to not begin with a separator.*/
  if (*src == '/') {
    src++;
  }

  /* Appending.*/
  while ((*dst++ = *src++) != '\0') {
    n++;

    if (n > size) {
      return (size_t)0;
    }
  }

  *dst = '\0';

  return n;
}

/**
 * @brief   Prepends a path to a path.
 *
 * @param[in, out]  dst         The destination path.
 * @param[in]       src         The path to be prepended.
 * @param[in]       size        Destination buffer size.
 * @return                      Offset in the buffer of the original path.
 * @retval 0                    Path error or buffer overflow.
 */
size_t path_prepend(char *dst, const char *src, size_t size) {
  size_t dn, sn;
  char *buf = dst;

  dn = strnlen(dst, size - 1U);
  sn = strnlen(src, size - 1U);

  /* The source path needs to end with a separator in case the destination
     does not have it. Note, the separator is not written here.*/
  if ((sn == 0U) || (src[sn - 1U] != '/')) {
    sn++;
  }

  /* If the destination has a separator then skipping it.*/
  if ((dn > 0U) && (dst[0] == '/')) {
    dn--;
    dst++;
  }

  if (dn + sn >= size) {
    return (size_t)0;
  }

  /* Making space for the prefix, including the final zero in the move.*/
  memmove(buf + sn, dst, dn + 1U);

  /* Placing the prefix omitting the final zero then enforcing the separator.*/
  memmove(buf, src, sn);
  buf[sn - 1U] = '/';

  return sn;
}

/**
 * @brief   Adds a separator to the end of a path if it is missing.
 *
 * @param[in, out]  dst         The destination path.
 * @param[in]       size        Destination buffer size.
 * @return                      The size of the combined path.
 * @retval 0                    Path error or buffer overflow.
 */
size_t path_add_separator(char *dst, size_t size) {
  size_t dn;

  dn = strnlen(dst, size - 1U);

  if (dn == 0U) {
    dst[0] = '/';
    dst[1] = '\0';
    dn++;
  }
  else {
    if (dst[dn - 1U] != '/') {
      if (dn >= size - 1U) {
        return (size_t)0;
      }

      dst[dn]     = '/';
      dst[dn + 1U] = '\0';
      dn++;
    }
  }

  return dn;
}

/**
 * @brief   Adds a file extension to a path, if not present.
 *
 * @param[in, out]  dst         The destination path.
 * @param[in]       ext         The extension string, must include the dot.
 * @param[in]       size        Destination buffer size.
 * @return                      The size of the combined path.
 * @retval 0                    Path error or buffer overflow.
 */
size_t path_add_extension(char *dst, const char *ext, size_t size) {
  size_t dn, en;

  dn = strlen(dst);
  en = strlen(ext);
  if ((dn < en) || (strcmp(dst + dn - en, ext) != 0)) {
    if (dn + en >= size) {
      return 0U;
    }
    memmove(dst + dn, ext, en + 1U);
  }

  return dn + en;
}

/**
 * @brief   Fetches the next path element.
 * @note    Does not consume the next separator, if any.
 * @note    Does not add a final zero to the fetched fname.
 * @note    It can return an empty element, it has to be detected outside.
 *
 * @param[in, out]  pathp       Pointer to the path under parsing.
 * @param[out]      dst         Buffer for the extracted path element.
 * @param[in]       size        Destination buffer size.
 * @return                      The size of the fetched path element, it does
 *                              not fetch beyond @p size.
 * @retval 0                    Null element.
 * @retval size                 Buffer overflow.
 */
size_t path_copy_element(const char **pathp, char *dst, size_t size) {
  size_t n;
  const char *p;

  p = *pathp;
  n = (size_t)0;
  while (true) {
    char c = *p;

    /* Path elements must be terminated by a separator or an end-of-string.*/
    if ((c == '/') || (c == '\0')) {

      /* Advancing the path pointer past the file name in the path and
         closing the file name string.*/
      *pathp = p;
      return n;
    }

    n++;

    /* Exceeding the maximum length considering the space for the final zero.*/
    if (n >= size) {
      return n;
    }

    *dst++ = c;
    p++;
  }
}

/**
 * @brief   Fetches the next path element.
 * @note    Does not consume the next separator, if any.
 * @note    It can return an empty element, it has to be detected outside.
 *
 * @param[in, out]  pathp       Pointer to the path under parsing.
 * @param[out]      dst         Buffer for the extracted path element.
 * @param[in]       size        Destination buffer size.
 * @return                      The size of the fetched path element, it does
 *                              not fetch beyond @p size.
 * @retval 0                    Null element.
 * @retval size                 Buffer overflow.
 */
size_t path_get_element(const char **pathp, char *dst, size_t size) {
  size_t n;

  n = path_copy_element(pathp, dst, size);
  if (n < size) {
    dst[n] = '\0';
  }

  return n;
}

/**
 * @brief   Verifies that the next path element is equal to the match string.
 *
 * @param[in]       path        The source path.
 * @param[in]       match       String to be matched.
 * @param[in]       size        Destination buffer size.
 * @return                      The size of the matched path element, it does
 *                              not match beyond @p size.
 * @retval 0                    Null element.
 * @retval size                 Buffer overflow or no match.
 */
size_t path_match_element(const char *path, const char *match, size_t size) {
  size_t n;

  n = (size_t)0;
  while (true) {
    char c1 = *path;
    char c2 = *match;

    /* Path elements must be terminated by a separator or an end-of-string.*/
    if ((c1 == '/') || (c1 == '\0')) {
      return c2 == '\0' ? n : size;
    }

    if (c1 != c2) {
      return size;
    }

    path++;
    match++;
    n++;

    /* Exceeding the maximum length considering the space for the final zero.*/
    if (n >= size) {
      return size;
    }
  }
}

/**
 * @brief   Normalizes an absolute path.
 * @note    The destination buffer can be the same of the source buffer.
 *
 * @param[out] dst              The destination buffer.
 * @param[in] src               The source path, must be absolute.
 * @param[in[ size              Destination buffer size.
 * @return                      The size of the normalized path.
 * @retval 0                    Path error.
 */
size_t path_normalize(char *dst, const char *src, size_t size) {
  size_t n;

  if (*src++ != '/') {
    return 0;
  }

  *dst++ = '/';
  n = 1U;
  while (true) {
    size_t ret;

    /* Consecutive input separators are consumed.*/
    while (*src == '/') {
      src++;
    }

    /* Getting next element from the input path and copying it to
       the output path.*/
    ret = path_copy_element(&src, dst, size - n);
    if (ret >= size - n) {
      return (size_t)0;
    }

    /* No next element condition.*/
    if ((size_t)ret == 0U) {
      /* If the path contains something after the root separator.*/
      if (n > 1U) {
        /* No next path element, replacing the last separator with a zero.*/
        n--;
        *(dst - 1) = '\0';
      }
      else {
        *dst = '\0';
      }

      return n;
    }

    /* Handling special cases of "." and "..".*/
    if (strncmp(dst, "..", 2U) == 0) {
      /* Double dot elements require to remove the last element from
         the output path.*/
      if (n > 1U) {
        /* Back on the separator.*/
        dst--;
        n--;

        /* Scanning back to just after the previous separator.*/
        do {
          dst--;
          n--;
        } while(*(dst - 1) != '/');
      }
      continue;
    }
    else if (strncmp(dst, ".", 1U) == 0) {
      /* Single dot elements are discarded.*/
      /* Consecutive input separators are consumed.*/
      continue;
    }

    dst += (size_t)ret;
    n   += (size_t)ret;

    /* Adding a single separator to the output.*/
    *dst++ = '/';
    n++;
  }
}

/** @} */
