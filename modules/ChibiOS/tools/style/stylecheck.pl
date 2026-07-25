#!/usr/bin/perl
use strict;
use warnings;
use File::Basename;

# Desired indentation.
my $indentation = 2;

if ($#ARGV != 0) {
  print "\nUsage: stylecheck.pl source\n";
  exit;
}

my $source = $ARGV[0];

open(my $in,  "<", $source) or die "Can't open source: $!";

my $lineno = 0;
my @c_source = <$in>;
my $filename = $source;
$filename =~ y/\\/\//;
$filename = basename($filename);

my $cr        = "\r";
my $lf        = "\n";
my $tab       = "\t";

sub style {
  my $desc = shift;

  print("style: $desc at line $lineno in \"$source\"\n");
}

sub error {
  my $desc = shift;

  print("error: $desc at line $lineno in \"$source\"\n");
}

my $emptycnt = 0;
my $c_comment_complete = 0;
my $c_comment = "";
my $state     = "start";
foreach my $line (@c_source) {

  $lineno += 1;

  #****************************************************************************
  # Processing comments after decoding.
  if ($c_comment_complete != 0) {
#    print($c_comment . "\n");
    
    #******************************************************************************
    # Special case of lint comment.
    if ("$c_comment" =~ /^\s*\/\*lint/) {
    }
    else {
      #******************************************************************************
      # Check on glued doxygen back-comment start.
      if ("$c_comment" =~ /^\s*\/\*\*<[^\s]/) {
        style "detected glued doxygen back-comment start";
      }

      #******************************************************************************
      # Check on glued doxygen comment start.
      if ("$c_comment" =~ /^\s*\/\*\*[^\s<]/) {
        style "detected glued doxygen comment start";
      }

      #******************************************************************************
      # Check on glued comment start.
      if ("$c_comment" =~ /^\s*\/\*[^\s\*=]/) {
        style "detected glued comment start";
      }

      #******************************************************************************
      # Check on lower case letter at comment beginning.
      if ("$c_comment" =~ /^\s*\/\*\s*[a-z]/) {
        style "detected lower case comment start";
      }

      #******************************************************************************
      # Check on loose comment stop.
#      if ("$line" =~ /\s\*\//) {
#        style "detected loose comment stop";
#      }
    }

    $c_comment_complete = 0;
  }

  #****************************************************************************
  # Check on EOL.
  if (not ($line =~ /$lf$/)) {
    error "detected malformed EOL";
  }
  $line =~ s/$cr//;
  $line =~ s/$lf//;

  #****************************************************************************
  # Check on trailing spaces.
  if ($line =~ /\s$/) {
    style "detected trailing spaces";
  }

  #****************************************************************************
  # Check on TABs.
  if ($line =~ /$tab/) {
    style "detected TAB";
    $line =~ s/$tab/    /;
  }

  #****************************************************************************
  # Check on empty lines.
  my $tmp = $line;
  $tmp =~ s/\s//;
  if (length($tmp) == 0) {
    $emptycnt = $emptycnt + 1;
    if ($emptycnt == 2) {
      style "detected multiple empty lines"
    }
    next;
  }
  else {
    $emptycnt = 0;
  }

  #****************************************************************************
  # Stripping strings content for ease of parsing, all strings become _string_.
  $line =~ s/\\\"//;
  if ($line =~ s/(\"[^"]*\")/_string_/) {
#    print "string: $1 replaced by _string_\n";
  }

  #******************************************************************************
  # State machine handling.
  if ($state eq "start") {

    #******************************************************************************
    # Standard separator.

    #******************************************************************************
    # Comment start matching.
    if ("$line" =~ /\/\*/) {
    
      #******************************************************************************
      # Single or multi line comments.
      if ("$line" =~ /\*\//) {
        # Special case of single line comments.
        $line =~ /(\/\*.*\*\/)/;
        $c_comment = $1;
        $c_comment_complete = 1;
      }
      else {
        # Start of multi-line comment.
        $line =~ /(\/\*.*)/;
        $c_comment = $1;
        $state = "incomment";
      }
    }
    else {

      #****************************************************************************
      # Check on C++ comments.
      if ($line =~ /\/\//) {
        style "detected // comment";
      }

      #****************************************************************************
      # Check on commas.
      if ($line =~ /,\S/) {
        style "detected comma not followed by space";
      }

      #****************************************************************************
      # Check on loose semicolons.
      if ($line =~ /\S\s;/) {
        style "detected loose semicolon";
      }

      #****************************************************************************
      # Check on glued keywords.
      if ($line =~ /\sif\(/) {
        style "detected glued \"if\"";
      }
      if ($line =~ /\sfor\(/) {
        style "detected glued \"for\"";
      }
      if ($line =~ /\swhile\(/) {
        style "detected glued \"while\"";
      }
      if ($line =~ /\)while/) {
        style "detected glued \"while\"";
      }
      if ($line =~ /\sswitch\(/) {
        style "detected glued \"switch\"";
      }
      if ($line =~ /\sdo\{/) {
        style "detected glued \"do\"";
      }

      #****************************************************************************
      # Check on loose parenthesis.
      if ($line =~ /\(\s+/) {
        style "detected loose \"(\"";
      }
      if ($line =~ /\S\s+\)/) {
        style "detected loose \")\"";
      }

      #****************************************************************************
      # Check on glued braces.
      if ($line =~ /\)\{/) {
        style "detected glued left brace";
      }
      if ($line =~ /\w\{/) {
        style "detected glued left brace";
      }
      if ($line =~ /\}\w/) {
        style "detected glued right brace";
      }

      #****************************************************************************
      # Check on (some) operators.
      # Before: <<= << >>= >> <= >= == != += -= *= /= %= &= |= ^=
      if ($line =~ /(\(\S<<=?|\S>>=?|[^\s<]<=|[^\s>]>=|\S[=!+\-*\/%&|^]=)/) {
        style "detected glued operator (1)";
      }
      # After: =
      elsif ($line =~ /=[^\s=]/) {
        style "detected glued assignment/comparison operator (2)";
      }
      # Before: =
      elsif ($line =~ /[^\s\=\!\+\-\*\/\%\&\|\^\<\>]=/) {
        style "detected glued assignment/comparison operator (3)";
      }
      # After: << >>
      elsif ($line =~ /(<<|>>)[^\s=]/) {
        style "detected glued assignment/comparison operator (4)";
      }
      # Before: && || ^^
      elsif ($line =~ /\S(&&|\|\||\^\^)/) {
        style "detected glued logical operator (1)";
      }
      # After: && || ^^
      elsif ($line =~ /(&&|\|\||\^\^)\S/) {
        style "detected glued logical operator (2)";
      }

      #****************************************************************************
      # Check function-call-like returns (not perfect so disabled).
      if ($line =~ /return\s*\(/) {
        if ($line =~ /return\s*\([\w\d\s\*]*\)\s*[^;]/) {
        }
        else {
#          style "detected function-call-like return";
        }
      }
    }
  }

  #******************************************************************************
  # Scanning for comment end.
  elsif ($state eq "incomment") {
    # Left trimming.
    $line =~ s/^\s+//;
    if ("$line" =~ /^\s*\*\/\s*$/) {
      # Just end of comment line.
      $c_comment .= "*/";
      $c_comment_complete = 1;
#      print("$c_comment");
      $state = "start";
    }
    elsif ("$line" =~ /\*\/\s*$/) {
      # Text followed by end of comment.
      $line =~ /(.*\*\/)/;
      $c_comment .= " " . $1;
      $c_comment_complete = 1;
#      print("$c_comment");
      $state = "start";
    }
    else {
      # Add the whole line, remove first * and following spaces if any.
      $line =~ s/^\*?\s*//;
      $c_comment .= " " . $line;
#      print("$c_comment");
    }
  }
}

close $in or die "$in: $!";
