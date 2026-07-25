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
 * @file    sglob.c
 * @brief   Simplified glob code.
 *
 * @addtogroup SGLOB
 * @{
 */

#include <stdbool.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <dirent.h>

#include "sglob.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/* Some GCC versions do not define this in newlib headers.*/
#if !defined(PATH_MAX)
#define PATH_MAX 1024
#endif

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

static bool isdots(const char *p) {

  if (p[0] == '.') {
    if (p[1] == '.') {
      if (p[2] == '\0') {
        return true;
      }
    }
    else if (p[1] == '\0') {
      return true;
    }
  }
  return false;
}

static char *lsalloc(sglob_t *psglob, size_t len) {
  lstring_t *lsp;

  lsp = malloc(len + sizeof (lstring_t));
  if (lsp == NULL) {
    return NULL;
  }

  lsp->next = NULL;
  if (psglob->sgl_hdr.next == NULL) {
    psglob->sgl_hdr.next = psglob->sgl_last = lsp;
  }
  else {
    psglob->sgl_last->next = lsp;
    psglob->sgl_last = lsp;
  }
  return lsp->string;
}

/* Copyright not specified so public domain code, not my thing.*/
static bool match(const char *pattern, const char *text) {

  if ((*pattern == '\0') && (*text == '\0'))
    return true;

  if ((*pattern == '*') && (*(pattern + 1) != '\0') && (*text == '\0'))
    return false;

  if ((*pattern == '?') || (*pattern == *text))
    return match(pattern + 1, text + 1);

  if (*pattern == '*')
    return match(pattern + 1, text) || match(pattern, text + 1);

  return false;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void sglob_init(sglob_t *psglob) {

  psglob->sgl_hdr.next  = NULL;
  psglob->sgl_last      = NULL;
  psglob->sgl_buf       = NULL;
  psglob->sgl_pathc     = 0;
  psglob->sgl_pathv     = NULL;
}

int sglob_add(sglob_t *psglob, const char *s) {
  char *p;
  int ret;

  p = lsalloc(psglob, strlen(s) + 1);
  if (p != NULL) {
    strcpy(p, s);
    psglob->sgl_pathc++;
    ret = 0;
  }
  else {
    ret = SGLOB_NOSPACE;
  }

  return ret;
}

int sglob_match(sglob_t *psglob, const char *pattern) {
  int ret;
  DIR *dbp;

  dbp = NULL;
  ret = SGLOB_NOSPACE;
  do {
    char *lastp, *dirp;
    size_t plen, dirlen;
    bool no_spec, found;
    struct dirent *dp;

    /* Checking the path length.*/
    plen = strnlen(pattern, PATH_MAX);
    if (plen >= PATH_MAX) {
      break;
    }

    /* Allocating a path buffer if not already done.*/
    if (psglob->sgl_buf == NULL) {
      psglob->sgl_buf = malloc(PATH_MAX);
      if (psglob->sgl_buf == NULL) {
        break;
      }
    }

    /* We need the path pattern in a writable buffer.*/
    strcpy(psglob->sgl_buf, pattern);

    /* Finding the last element, the one to be expanded.*/
    lastp = rindex(psglob->sgl_buf, '/');
    if (lastp == NULL) {
      /* It is all last element.*/
      lastp    = psglob->sgl_buf;
      dirp     = NULL;
      dirlen   = (size_t)0;
    }
    else {
      *lastp++ = '\0';
      dirp     = psglob->sgl_buf;
      dirlen   = strlen(dirp);
    }

    /* Case with no wildcards, adding with no processing.*/
    no_spec = (bool)((index(lastp, '?') == NULL) &&
                     (index(lastp, '*') == NULL));
    if (no_spec) {
      return sglob_add(psglob, pattern);
    }

    /* Opening the directory part of the path. If it faults then it is not
       reported as an error but an empty sglob is returned, this can be
       detected from outside.*/
    dbp = opendir(dirp == NULL ? "." : dirp);
    if (dbp == NULL) {
      ret = SGLOB_NOMATCH;
      break;
    }

    /* Scanning the directory for matches.*/
    found = false;
    while ((dp = readdir(dbp)) != NULL) {
      char *p;

      if (isdots(dp->d_name) || !match(lastp, dp->d_name)) {
        continue;
      }

      /* Inserting the new found match into the list.*/
      found = true;
      p = lsalloc(psglob, dirlen + strlen(dp->d_name) + 1);
      if (p == NULL) {
        goto skiperr;
      }
      *p = '\0';
      if (dirlen > 0) {
        strcat(p, dirp); /* TODO Optimize strcat */
        strcat(p, "/");
      }
      strcat(p, dp->d_name);
      psglob->sgl_pathc++;
    }

    if (found) {
      ret = 0;
    }
    else {
      ret = SGLOB_NOMATCH;
    }

skiperr:
    closedir(dbp);
  }
  while (false);

  return ret;
}

int sglob_build(sglob_t *psglob, size_t offs) {
  lstring_t *lsp;
  int i;

  psglob->sgl_pathv = malloc((psglob->sgl_pathc + offs + 1) * sizeof (char *));
  if (psglob->sgl_pathv == NULL) {
    return SGLOB_NOSPACE;
  }

  i = offs;
  lsp = psglob->sgl_hdr.next;
  while (lsp != NULL) {
    psglob->sgl_pathv[i++] = lsp->string;
    lsp = lsp->next;
  }
  psglob->sgl_pathv[i] = NULL;

  return 0;
}

void sglob_free(sglob_t *psglob) {
  lstring_t *lsp;

  if (psglob->sgl_buf != NULL) {
    free(psglob->sgl_buf);
  }

  if (psglob->sgl_pathv != NULL) {
    free(psglob->sgl_pathv);
  }

  while ((lsp = psglob->sgl_hdr.next) != NULL) {
    psglob->sgl_hdr.next = lsp->next;
    free(lsp);
  }
}

/** @} */
