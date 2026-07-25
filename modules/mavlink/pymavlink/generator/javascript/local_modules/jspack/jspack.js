/**
 * @license

 Copyright © 2008 Fair Oaks Labs, Inc.
  All rights reserved.

  This file is Modified from the original, by buzz 2020:
   - ran thru http://www.jsnice.org/ and manually renamed the variables to be clearer 
   - added optionally enabled debugging/verbose/printfs throughout
   - bugfixes and integration so it now passes our mavlink.js testsuite/s
   - please see README.md in the upper level folder.
*/
'use strict';

var Long = require('long');

let DEBUG = false;

/**
 * @return {undefined}
 */
function JSPack() {
  var el;
  /** @type {boolean} */
  var booleanIsBigEndian = false;
  var m = this;
  
  
  /**
   * @param {!Object} octet_array_a
   * @param {number} offset_p
   * @param {number} len
   * @return {?}
   */
  //Raw byte arrays
 // m._DeArray = function(octet_array_a, offset_p, len) {
 // if (DEBUG) console.log("zzz1");
 //   return [octet_array_a.slice(offset_p, offset_p + len)];
  //};
  
  /**
   * @param {!Array} to_octet_array_a
   * @param {number} offset_p
   * @param {number} len
   * @param {!NodeList} from_array_v
   * @return {undefined}
   */
 // m._EnArray = function(to_octet_array_a, offset_p, len, from_array_v) {
 // if (DEBUG) console.log("zzz2");
    /** @type {number} */
  //  var i = 0;
  //  for (; i < len; to_octet_array_a[offset_p + i] = from_array_v[i] ? from_array_v[i] : 0, i++) {
  //  }
  //};
  
  
  /**
   * @param {!Object} octet_array_a
   * @param {number} offset_p
   * @return {?}
   */
  // ASCII characters
  m._DeChar = function(octet_array_a, offset_p) {
  if (DEBUG) console.log("zzz3");
    return String.fromCharCode(octet_array_a[offset_p]);
  };
  /**
   * @param {!Array} to_octet_array_a
   * @param {number} offset_p
   * @param {string} from_str_array_v
   * @return {undefined}
   */
 // m._EnChar = function(to_octet_array_a, offset_p, from_str_array_v) {
 // if (DEBUG) console.log("zzz4");
 //   /** @type {number} */
 //   to_octet_array_a[offset_p] = from_str_array_v.charCodeAt(0);
 // };
  
  
  /**
   * @param {!Object} octet_array_a
   * @param {number} offset_p
   * @return {?}
   */
  //Little-endian (un)signed N-byte integers
  m._DeInt = function(octet_array_a, offset_p) {
  if (DEBUG) console.log("zzz5");
    /** @type {number} */
    var lsb = booleanIsBigEndian ? el.len - 1 : 0;
    /** @type {number} */
    var nsb = booleanIsBigEndian ? -1 : 1;
    /** @type {number} */
    var stop = lsb + nsb * el.len;
    var rv;
    var i;
    var f;
    /** @type {number} */
    rv = 0;
    /** @type {number} */
    i = lsb;
    /** @type {number} */
    f = 1;
    for (; i != stop; rv = rv + octet_array_a[offset_p + i] * f, i = i + nsb, f = f * 256) {
    }
    if (el.bSigned && rv & Math.pow(2, el.len * 8 - 1)) {
      /** @type {number} */
      rv = rv - Math.pow(2, el.len * 8);
    }
    return rv;
  };
  
  
  /**
   * @param {!Array} octet_array_a
   * @param {number} offset_p
   * @param {number} val
   * @return {undefined}
   */
  m._EnInt = function(octet_array_a, offset_p, val) {
  if (DEBUG) console.log("chunk-from: "+val);
    /** @type {number} */
    var lsb = booleanIsBigEndian ? el.len - 1 : 0;
    /** @type {number} */
    var nsb = booleanIsBigEndian ? -1 : 1;
    /** @type {number} */
    var stop = lsb + nsb * el.len;
    var i;
    // range limit:
    if (val < el.min ) {
        val = el.min;
        console.log("value limited to MIN:"+val);   
    }
    if (val > el.max ) {
        val =  el.max;
        console.log("value limited to MAX:"+val);   
    }
    /** @type {number} */
    i = lsb;
  if (DEBUG) console.log("booleanIsBigEndian:"+booleanIsBigEndian);   
  if (DEBUG) console.log("el.len:"+el.len);   
  if (DEBUG) console.log("lsb:"+lsb);   
  if (DEBUG) console.log("nsb:"+nsb);   
  if (DEBUG) console.log("i:"+i);   
  if (DEBUG) console.log("stop:"+stop);   
    for (; i != stop; ) {

         var to = JSON.stringify(val&255);
        if (DEBUG) console.log("chunk as bytes: "+to);

        octet_array_a[offset_p + i] = val & 255;
        i = i + nsb;
        val = val >> 8;


    }
  };
  
  
  /**
   * @param {!Object} octet_array_a
   * @param {number} offset_p
   * @param {number} len
   * @return {?}
   */
  // ASCII character strings
  m._DeString = function(octet_array_a, offset_p, len) {
  if (DEBUG) console.log("zzz7");
    /** @type {!Array} */
    var retval = new Array(len);
    /** @type {number} */
    var i = 0;
    for (; i < len; retval[i] = String.fromCharCode(octet_array_a[offset_p + i]), i++) {
    }
    return retval.join("");
  };
  /**
   * @param {!Array} octet_array_a
   * @param {number} offset_p
   * @param {number} len
   * @param {string} strval
   * @return {undefined}
   */
  m._EnString = function(octet_array_a, offset_p, len, strval) {
  if (DEBUG) console.log("zzz8");
    var t;
    /** @type {number} */
    if ( DEBUG ) console.log("strencode before: "+octet_array_a+"\np:"+offset_p+" len:"+len+" strval:"+strval) 
    var i = 0;
    //if (DEBUG) console.log("strval:"+strval);
//console.trace("Here I am!")

    // we all strings to be passed in as a string of characters, or a an array or buffer of them is ok too

    if (typeof strval.charCodeAt === "function") { 
      for (; i < len; octet_array_a[offset_p + i] = (t = strval.charCodeAt(i)) ? t : 0, i++) {
        if ( t > 255 ) console.log("ERROR ERROR ERROR ERROR ERROR ERROR - It seems u passed unicode/utf-8/etc to jspack, not 8 bit ascii. please use .toString('binary'); not .toString();");
      }
      if ( DEBUG ) console.log("strencode from CHAR-string."); 

    } else if (Array.isArray(strval)) { 
      for (; i < len; octet_array_a[offset_p + i] = (t = strval[i]) ? t : 0, i++) {
        // referring directly to 't' inside this loop is bad, seems delayed by an iteration, but strval[i] is ok.
        if ( strval[i] > 255 )  console.log("ERROR ERROR ERROR ERROR ERROR ERROR - It seems u passed unicode/utf-8/etc, or array data with values > 255, to jspack, not 8 bit ascii.\n(bad Array data)"+strval[i]);
      }
      if ( DEBUG ) console.log("strencode from ARRAY."); 

    } else if (Buffer.isBuffer(strval)) { 
      for (; i < len; octet_array_a[offset_p + i] = (t = strval[i]) ? t : 0, i++) {
        if ( strval[i] > 255 ) console.log("ERROR ERROR ERROR ERROR ERROR ERROR - It seems u passed unicode/utf-8/etc to jspack, not 8 bit ascii. \n(bad Buffer data)"+strval[i]);
      }
      if ( DEBUG ) console.log("strencode from Buffer."); 

    } else {
        console.log("ERROR encoding string _EnString:  array:"+octet_array_a+" p:"+offset_p+" len:"+len+" strval:"+JSON.stringify(strval)) 
}
  };
  
  
  /**
   * @param {!Object} octet_array_a
   * @param {number} offset_p
   * @return {?}
   */
  // Little-endian N-bit IEEE 754 floating point
  m._De754 = function(octet_array_a, offset_p) {
  if (DEBUG) console.log("zzz9");
    var bool_s;
    var exponent;
    var mantissa;
    var i;
    var d;
    var nBits;
    var mantissaLen;
    var exponentLen;
    var eBias;
    var eMax;
    mantissaLen = el.mLen;
    /** @type {number} */
    exponentLen = el.len * 8 - el.mLen - 1;
    /** @type {number} */
    eMax = (1 << exponentLen) - 1;
    /** @type {number} */
    eBias = eMax >> 1;
    /** @type {number} */
    i = booleanIsBigEndian ? 0 : el.len - 1;
    /** @type {number} */
    d = booleanIsBigEndian ? 1 : -1;
    bool_s = octet_array_a[offset_p + i];
    /** @type {number} */
    i = i + d;
    /** @type {number} */
    nBits = -7;
    /** @type {number} */
    exponent = bool_s & (1 << -nBits) - 1;
    /** @type {number} */
    bool_s = bool_s >> -nBits;
    /** @type {number} */
    nBits = nBits + exponentLen;
    for (; nBits > 0; exponent = exponent * 256 + octet_array_a[offset_p + i], i = i + d, nBits = nBits - 8) {
    }
    /** @type {number} */
    mantissa = exponent & (1 << -nBits) - 1;
    /** @type {number} */
    exponent = exponent >> -nBits;
    nBits = nBits + mantissaLen;
    for (; nBits > 0; mantissa = mantissa * 256 + octet_array_a[offset_p + i], i = i + d, nBits = nBits - 8) {
    }
    switch(exponent) {
      case 0:
        /** @type {number} */
        // Zero, or denormalized number
        exponent = 1 - eBias;
        break;
      case eMax:
        // NaN, or +/-Infinity
        return mantissa ? NaN : (bool_s ? -1 : 1) * Infinity;
      default:
        // Normalized number
        mantissa = mantissa + Math.pow(2, mantissaLen);
        /** @type {number} */
        exponent = exponent - eBias;
        break;
    }
    return (bool_s ? -1 : 1) * mantissa * Math.pow(2, exponent - mantissaLen);
  };
  /**
   * @param {!Array} octet_array_a
   * @param {number} offset_p
   * @param {number} v
   * @return {undefined}
   */
  m._En754 = function(octet_array_a, offset_p, v) {
  if (DEBUG) console.log("zzz_10");
    var bool_s;
    var exponent;
    var mantissa;
    var i;
    var d;
    var c;
    var mantissaLen;
    var exponentLen;
    var eBias;
    var eMax;
    mantissaLen = el.mLen;
    /** @type {number} */
    exponentLen = el.len * 8 - el.mLen - 1;
    /** @type {number} */
    eMax = (1 << exponentLen) - 1;
    /** @type {number} */
    eBias = eMax >> 1;
    /** @type {number} */
    bool_s = v < 0 ? 1 : 0;
    /** @type {number} */
    v = Math.abs(v);
    if (isNaN(v) || v == Infinity) {
      /** @type {number} */
      mantissa = isNaN(v) ? 1 : 0;
      /** @type {number} */
      exponent = eMax;
    } else {
      /** @type {number} */
      exponent = Math.floor(Math.log(v) / Math.LN2);// Calculate log2 of the value
      if (v * (c = Math.pow(2, -exponent)) < 1) { // Math.log() isn't 100% reliable
        exponent--;
        /** @type {number} */
        c = c * 2;
      }
      // Round by adding 1/2 the significand's LSD
      if (exponent + eBias >= 1) {
        /** @type {number} */
        v = v + el.rt / c; // Normalized:  mLen significand digits
      } else {
        /** @type {number} */
        v = v + el.rt * Math.pow(2, 1 - eBias);// Denormalized:  <= mLen significand digits
      }
      if (v * c >= 2) {
        exponent++;
        /** @type {number} */
        c = c / 2; 	// Rounding can increment the exponent
      }
      if (exponent + eBias >= eMax) {
        // Overflow
        /** @type {number} */
        mantissa = 0;
        /** @type {number} */
        exponent = eMax;
      } else {
        if (exponent + eBias >= 1) {
          	// Normalized - term order matters, as Math.pow(2, 52-e) and v*Math.pow(2, 52) can overflow
          /** @type {number} */
          mantissa = (v * c - 1) * Math.pow(2, mantissaLen);
          /** @type {number} */
          exponent = exponent + eBias;
        } else {
          // Denormalized - also catches the '0' case, somewhat by chance
          /** @type {number} */
          mantissa = v * Math.pow(2, eBias - 1) * Math.pow(2, mantissaLen);
          /** @type {number} */
          exponent = 0;
        }
      }
    }
    /** @type {number} */
    i = booleanIsBigEndian ? el.len - 1 : 0;
    /** @type {number} */
    d = booleanIsBigEndian ? -1 : 1;
    for (; mantissaLen >= 8; octet_array_a[offset_p + i] = mantissa & 255, i = i + d, mantissa = mantissa / 256, mantissaLen = mantissaLen - 8) {
    }
    /** @type {number} */
    exponent = exponent << mantissaLen | mantissa;
    exponentLen = exponentLen + mantissaLen;
    for (; exponentLen > 0; octet_array_a[offset_p + i] = exponent & 255, i = i + d, exponent = exponent / 256, exponentLen = exponentLen - 8) {
    }
    octet_array_a[offset_p + i - d] |= bool_s * 128;
  };
  
  
  /**
   * @param {!Object} octet_array_a
   * @param {number} offset_p
   * @return {?}
   */
  	// Convert int64 to array with 3 elements: [lowBits, highBits, unsignedFlag]
	// '>>>' trick to convert signed 32bit int to unsigned int (because << always results in a signed 32bit int)
  m._DeInt64 = function(octet_array_a, offset_p) {
  if (DEBUG) console.log("zzz_11");
    /** @type {number} */
    var lsb = booleanIsBigEndian ? 0 : 7;
    /** @type {number} */
    var nsb = booleanIsBigEndian ? 1 : -1;
    /** @type {number} */
    var stop = lsb + nsb * 8;
    /** @type {!Array} */
    var nextIdLookup = [0, 0, !el.bSigned];
    var i;
    var f;
    var indexLookupKey;
    /** @type {number} */
    i = lsb;
    /** @type {number} */
    indexLookupKey = 1;
    /** @type {number} */
    f = 0;
    for (; i != stop; nextIdLookup[indexLookupKey] = (nextIdLookup[indexLookupKey] << 8 >>> 0) + octet_array_a[offset_p + i], i = i + nsb, f++, indexLookupKey = f < 4 ? 1 : 0) {

    if ( DEBUG ) console.log("jsPacking int64:"+octet_array_a[offset_p + i]);

    }
    return nextIdLookup;
  };
  /**
   * @param {!Array} octet_array_a
   * @param {number} offset_p
   * @param {!Object} v
   * @return {undefined}
   */
  m._EnInt64 = function(octet_array_a, offset_p, v) {

  if (v.length != 2) { //todo put this error back
     console.log("ERROR ERROR: jspack needs an array of at least length TWO to pack an int64 "+v+' len:'+v.length); 
  } 
//  if (DEBUG) console.log("zzz_12 v:"+v);
    /** @type {number} */
    var lsb = booleanIsBigEndian ? 0 : 7;
    /** @type {number} */
    var nsb = booleanIsBigEndian ? 1 : -1;
    /** @type {number} */
    var stop = lsb + nsb * 8;
    var i;
    var f;
    var j;
    var shift;
    /** @type {number} */
    i = lsb;
    /** @type {number} */
    j = 1;
    /** @type {number} */
    f = 0;
    /** @type {number} */
    shift = 24;

    for (; i != stop; octet_array_a[offset_p + i] = v[j] >> shift & 255, i = i + nsb, f++, j = f < 4 ? 1 : 0, shift = 24 - 8 * (f % 4)) {
    var x = v[j] >> shift & 255 ; 
    var vj = v[j];

    if ( DEBUG )  console.log('js qqqq  vj:'+vj+' j:'+j+' x:'+x+' a:'+octet_array_a+' i:'+i+" offset_p:"+offset_p+" v:"+v); 
    }
  };
  
  
  
  	// Class data
  /** @type {string} */
  m._sPattern = "(\\d+)?([AxcbBhHsfdiIlLqQ])";

  m._lenLut	= {'A':1, 'x':1, 'c':1, 'b':1, 'B':1, 'h':2, 'H':2, 's':1, 'f':4, 'd':8, 'i':4, 'I':4, 'l':4, 'L':4, 'q':8, 'Q':8};
  
  m._elLookUpTable	= {	'A': {en:m._EnArray, de:m._DeArray},
				's': {en:m._EnString, de:m._DeString},
				'c': {en:m._EnChar, de:m._DeChar},
				'b': {en:m._EnInt, de:m._DeInt, len:1, bSigned:true, min:-Math.pow(2, 7), max:Math.pow(2, 7)-1},
				'B': {en:m._EnInt, de:m._DeInt, len:1, bSigned:false, min:0, max:Math.pow(2, 8)-1},
				'h': {en:m._EnInt, de:m._DeInt, len:2, bSigned:true, min:-Math.pow(2, 15), max:Math.pow(2, 15)-1},
				'H': {en:m._EnInt, de:m._DeInt, len:2, bSigned:false, min:0, max:Math.pow(2, 16)-1},
				'i': {en:m._EnInt, de:m._DeInt, len:4, bSigned:true, min:-Math.pow(2, 31), max:Math.pow(2, 31)-1},
				'I': {en:m._EnInt, de:m._DeInt, len:4, bSigned:false, min:0, max:Math.pow(2, 32)-1},
				'l': {en:m._EnInt, de:m._DeInt, len:4, bSigned:true, min:-Math.pow(2, 31), max:Math.pow(2, 31)-1},
				'L': {en:m._EnInt, de:m._DeInt, len:4, bSigned:false, min:0, max:Math.pow(2, 32)-1},
				'f': {en:m._En754, de:m._De754, len:4, mLen:23, rt:Math.pow(2, -24)-Math.pow(2, -77)},
				'd': {en:m._En754, de:m._De754, len:8, mLen:52, rt:0},
				'q': {en:m._EnInt64, de:m._DeInt64, bSigned:true, len:8  },   // 64bit fields need 8 bytes..
				'Q': {en:m._EnInt64, de:m._DeInt64, bSigned:false, len:8 }}; // quirk of longs is they come in with a length of 2 in an array
  
  
  /**
   * @param {number} num_elements_n
   * @param {number} size_s
   * @param {!Object} octet_array_a
   * @param {number} offset_p
   * @return {?}
   */
  	// Unpack a series of n elements of size s from array a at offset p with fxn
  m._UnpackSeries = function(num_elements_n, size_s, octet_array_a, offset_p) {
  if (DEBUG) console.log("zzz_13");
    var fxn = el.de;
    /** @type {!Array} */
    var rv = [];
    /** @type {number} */
    var o = 0;
    for (; o < num_elements_n; rv.push(fxn(octet_array_a, offset_p + o * size_s)), o++) {
    }
    return rv;
  };
  /**
   * @param {number} num_elements_n
   * @param {number} size_s
   * @param {!Array} to_octet_array_a
   * @param {number} array_a_offset_p
   * @param {(Array|NodeList|null)} from_array_v
   * @param {number} array_v_offset_i
   * @return {undefined}
   */
  	// Pack a series of n elements of size s from array v at offset i to array a at offset p with fxn
  
  m._PackSeries = function(num_elements_n, size_s, to_octet_array_a, array_a_offset_p, from_array_v, array_v_offset_i) {
  if (DEBUG) console.log("pack-series: ");


    if ( DEBUG ) console.log('js before  0:'+0+' num_elements_n:'+num_elements_n+' size_s:'+size_s+' to_a:'+to_octet_array_a+' i:'+array_v_offset_i+" offset_p:"+array_a_offset_p+" v:"+from_array_v); 
    var fxn = el.en;
    /** @type {number} */
    var o = 0;
    for (; o < num_elements_n;  o++) {
        //if (DEBUG) console.log("14 called fxn with o:"+o);
        var z = from_array_v[array_v_offset_i + o]; 
        var to = JSON.stringify(z);
        var too = JSON.stringify(from_array_v);
          if (DEBUG) console.log('js pre-ffff  z:'+z+' to:'+to+' too:'+too+'');
        // handle flattened arrays - non-array things don't have a .length
        try {
        if (z.length == undefined ) {
                    //from_array_v = [ from_array_v ] ; 
                      if (DEBUG) console.log('Z FIX');
        }} catch (e){}
        var z = from_array_v[array_v_offset_i + o]; 
        var to = JSON.stringify(z);
        var too = JSON.stringify(from_array_v);

        // if we only have one thing to back and its got an 8 byte target len ( it's a 64bit long),  and length of source array is 2 ( low and high bits ) 
        // we treat it as a singular thing... we use this for Q type, which gets passed in as [lowBits, hightBits]
        if (( num_elements_n == 1 ) && (size_s == 8) && (from_array_v.length == 2) ) {  
            z = from_array_v; 
            if (DEBUG) console.log("js handling Q 64bit array"); 
        } 


        if (DEBUG) console.log('js partial  z:'+z+' to:'+to+' too:'+too+' num_elements_n:'+num_elements_n+' size_s:'+size_s+' to_a:'+to_octet_array_a+' v_offset_i:'+array_v_offset_i+" a_offset_p:"+array_a_offset_p+" from_v:"+from_array_v); 

        fxn(to_octet_array_a, array_a_offset_p + o * size_s, z);

    }
        if (DEBUG) console.log('js after  to_a:'+to_octet_array_a);
  };
  
  
  /**
   * @param {string} fmt
   * @param {!Object} octet_array_a
   * @param {number} offset_p
   * @return {?}
   */
  	// Unpack the octet array a, beginning at offset p, according to the fmt string
  m.Unpack = function(fmt, octet_array_a, offset_p) {
  if (DEBUG) console.log("zzz_15");
    /** @type {boolean} */
    // Set the private bBE flag based on the format string - assume big-endianness
    booleanIsBigEndian = fmt.charAt(0) != "<";
    /** @type {number} */
    offset_p = offset_p ? offset_p : 0;
    /** @type {!RegExp} */
    var re = new RegExp(this._sPattern, "g");
    var re_match;
    var repeat_count_n;
    var element_size_s;
    /** @type {!Array} */
    var rv = [];
    
    //loop over chars in the format string with regex due to optional digits
    for (; re_match = re.exec(fmt);) {
      /** @type {number} */
      repeat_count_n = re_match[1] == undefined || re_match[1] == "" ? 1 : parseInt(re_match[1]);
      element_size_s = this._lenLut[re_match[2]];
      if (offset_p + repeat_count_n * element_size_s > octet_array_a.length) {
        return undefined;
      }
      switch(re_match[2]) {
        case "A":
        case "s":
          rv.push(this._elLookUpTable[re_match[2]].de(octet_array_a, offset_p, repeat_count_n));
          break;
        case "c":
        case "b":
        case "B":
        case "h":
        case "H":
        case "i":
        case "I":
        case "l":
        case "L":
        case "f":
        case "d":
        case "q":
        case "Q":
          el = this._elLookUpTable[re_match[2]];
          
          //rv.push(this._UnpackSeries(repeat_count_n, element_size_s, octet_array_a, offset_p));

          // unpack arrays to an actual array type within the field array result:
          // https://github.com/AndreasAntener/node-jspack/commit/4f16680101303a6b4a1b0deba8cf7d20fc68213e
          if (repeat_count_n > 1) {
            // Field is array, unpack into separate array and push as such
            var arr = [];
            arr.push(this._UnpackSeries(repeat_count_n, element_size_s, octet_array_a, offset_p));
            rv.push(arr);
          } else {
            rv.push(this._UnpackSeries(repeat_count_n, element_size_s, octet_array_a, offset_p));
          }

          break;
      }
      /** @type {number} */
      offset_p = offset_p + repeat_count_n * element_size_s;
    }
    return Array.prototype.concat.apply([], rv);
  };
  

  // cross check the list of input data matches the size of bytes we'll be assembling
  // this is a slightly tweaked implementation of the previous 'PackTo' commented out below.
  // it has a more-consistent approach to input and output arrays, paying particular attention to Q,q, long, etc
  m.WouldPack = function(fmt, octet_array_a, offset_p, values) {
    //if (DEBUG) console.log("zzz_16 fmt:"+JSON.stringify(fmt)+" values:"+JSON.stringify(values));
    // @type {boolean} /
    // Set the private bBE flag based on the format string - assume big-endianness
    booleanIsBigEndian = fmt.charAt(0) != "<";
    // @type {!RegExp} /
    var re = new RegExp(this._sPattern, "g");

    var m;
    var n;
    var s;
    var values_i = 0; // current index into the values[] 

    var j;
    for (; m = re.exec(fmt);) {

        // leading optional prefix num or 1
        n = m[1] == undefined || m[1] == "" ? 1 : parseInt(m[1]);

        s = this._lenLut[m[2]];


        if (DEBUG) console.log("character: "+m[2]+"  how many(n)?: "+n);
        el = this._elLookUpTable[m[2]];

        //if (DEBUG) console.log("using lookup table:"+JSON.stringify(el));
        var bytes_consumed_per_element = el["len"];
        bytes_consumed_per_element = bytes_consumed_per_element == undefined ? 1 : bytes_consumed_per_element ; // undefined means 1
        if (DEBUG) console.log("bytes_consumed_per_element:"+JSON.stringify(bytes_consumed_per_element));
        if (DEBUG) console.log("current_values_idx:"+JSON.stringify(values_i) +" values:"+JSON.stringify(values[values_i])  )  ;


        // do per-case behaviours  'A' , 's' and 'x' are special, everything else gets the same
        switch(m[2]) {
        //------------------------------------------
        case "A":
        case "s":
          if (values_i + 1 > values.length) {
            console.log("JSPACK-ERROR: values_i + 1 > values.length  values_i:"+values_i+" values.length:"+values.length); 
            //return false;
          }
          if (DEBUG) console.log("all values:"+JSON.stringify(values));
          this._elLookUpTable[m[2]].en(octet_array_a, offset_p, n, values[values_i]);
          // @type {number} /
          values_i = values_i + 1;
          break;
        //------------------------------------------
        case "x":
          // @type {number} /
          j = 0;
          for (; j < n; j++) {
            // @type {number} /
            octet_array_a[offset_p + j] = 0;
          }
          break;
        //------------------------------------------
        // everything else
        default:

          // if n > 1 , ie it's multiple occurrences of a 'thing'
          if (n > 1 ) {

            // if we were handed an array at this idx, we need the array to be the same length as n
            if (Array.isArray(values[values_i])) {

                // Value series is array, iterate through that, only increment by 1
                if ((values_i + 1) > values.length) { 
                if (DEBUG) console.log("JSPACK-ERROR: value series is array but (values_i + 1) > values.length. i:"+values_i+" values.length:"+values.length); 
                //return false; 
                }
                  if (DEBUG) console.log("(dst IS array) (source IS array)");
                this._PackSeries(n, s, octet_array_a, offset_p, values[values_i], 0);
                values_i += 1;
            }
             else {
              if (DEBUG) console.log("ERROR: (dst IS array) (source is not array)");            
            }
          } 

          // if n == 1, it just one of a thing
          if (n == 1 ) {

          // type Q can have the source as an array when there is only 1 of them.
            if (Array.isArray(values[values_i]) ) {

                if (( m[2] == 'Q' ) || ( m[2] == 'q' ) ) {  
                this._PackSeries(n, s, octet_array_a, offset_p, values[values_i], 0);
                values_i += 1;
                }
                if (DEBUG) console.log("(dst is not array) (source IS array)");

            } else {
                  if ((values_i + n ) > values.length) { 
                        if (DEBUG) console.log("JSPACK-ERROR: value series NOT array but (values_i + n ) > values.length. i:"+values_i+" n:"+n+" values.length:"+values.length+" values:"+JSON.stringify(values));  
                        //return false; 
                  }
                  if (DEBUG) console.log("(dst is not array)  (source is not array)");
                  this._PackSeries(n, s, octet_array_a, offset_p, values, values_i);
                  values_i += n;
            }
          }

          if (DEBUG) console.log("");
          break;
        //------------------------------------------
        }

      offset_p = offset_p + n * s;

    }
    if (DEBUG) console.log("wouldpack completed, result array_a is:"+JSON.stringify(octet_array_a));
    return octet_array_a
  }



  /**
   * @param {string} fmt
   * @param {!Array} octet_array_a
   * @param {number} offset_p
   * @param {!NodeList} values
   * @return {?}
   */
/* 
  	// Pack the supplied values into the octet array a, beginning at offset p, according to the fmt string
  m.PackTo = function(fmt, octet_array_a, offset_p, values) {
  if (DEBUG) console.log("zzz_16 fmt:"+JSON.stringify(fmt)+" values:"+JSON.stringify(values));
    // @type {boolean} /
    // Set the private bBE flag based on the format string - assume big-endianness
    booleanIsBigEndian = fmt.charAt(0) != "<";
    // @type {!RegExp} /
    var re = new RegExp(this._sPattern, "g");
    var m;
    var n;
    var s;
    // @type {number} /
    var i = 0;
    var j;
    for (; m = re.exec(fmt);) {
      // @type {number} /
      n = m[1] == undefined || m[1] == "" ? 1 : parseInt(m[1]);
      s = this._lenLut[m[2]];
      if (offset_p + n * s > octet_array_a.length) {
        console.log("JSPACK-ERROR: offset_p + n * s > octet_array_a.length  offset_p:"+offset_p+" n:"+n+" s:"+s+" octet_array_a.length:"+octet_array_a.length+" octet_array_a:"+JSON.stringify(octet_array_a));  
        return false;
      }
      if (DEBUG) console.log("\n---------------------------------------------\n");
      if (DEBUG) console.log("handling format specifier:"+m[2]+"  how many:"+n);
      switch(m[2]) {
        case "A":
        case "s":
          if (i + 1 > values.length) {
            console.log("JSPACK-ERROR: i + 1 > values.length  i:"+i+" values.length:"+values.length); 
            return false;
          }
          if (DEBUG) console.log("zzz_16A values:"+JSON.stringify(values));
          this._elLookUpTable[m[2]].en(octet_array_a, offset_p, n, values[i]);
          // @type {number} /
          i = i + 1;
          break;
        case "c":
        case "b":
        case "B":
        case "h":
        case "H":
        case "i":
        case "I":
        case "l":
        case "L":
        case "f":
        case "d":
        case "q":
        case "Q":
          if (DEBUG) console.log("16 blerg");
          el = this._elLookUpTable[m[2]];
          if (DEBUG) console.log("using lookup table:"+JSON.stringify(el));
          //if (i + n > values.length) { return false;  }
          //this._PackSeries(n, s, octet_array_a, offset_p, values, i);
          //i = i + n;
          //added support for packing value series when they are supplied as arrays within the values array
          // https://github.com/AndreasAntener/node-jspack/commit/8de80d20aa06dea15527b3073c6c8631abda0f17
          if (n > 1 && Array.isArray(values[i])) {
            // Value series is array, iterate through that, only increment by 1
            if ((i + 1) > values.length) { 
              console.log("JSPACK-ERROR: value series is array but (i + 1) > values.length. i:"+i+" values.length:"+values.length); 
              return false; 
            }
            if (DEBUG) console.log("zzz_16 option 1 (source is array)");
            this._PackSeries(n, s, octet_array_a, offset_p, values[i], 0);
            i += 1;
          } else {
              if ((i + n) > values.length) { 
                    console.log("JSPACK-ERROR: value series NOT array but (i + n) > values.length. i:"+i+" n:"+n+" values.length:"+values.length+" values:"+JSON.stringify(values));  
                    //return false; 
              }
              if (DEBUG) console.log("zzz_16 option 2 (source is not array)");
            this._PackSeries(n, s, octet_array_a, offset_p, values, i);
            i += n;
          }

          if (DEBUG) console.log("end case");
          break;
        case "x":
          // @type {number} /
          j = 0;
          for (; j < n; j++) {
            // @type {number} /
            octet_array_a[offset_p + j] = 0;
          }
          break;
      }
      // @type {number} /
      offset_p = offset_p + n * s;
    }
    console.log("pack completed, result array_a is:"+JSON.stringify(octet_array_a));
    return octet_array_a;
  };
  */
  
  /**
   * @param {string} fmt
   * @param {(Node|NodeList|null|string)} values
   * @return {?}
   */
  	// Pack the supplied values into a new octet array, according to the fmt string
  m.Pack = function(fmt, values) {
  if (DEBUG) console.log("\n\n------------------------------------------------------------------------------------------------------------\n\n");
  if (DEBUG) console.log("initial unpacked values:"+JSON.stringify(values));
  if (DEBUG) console.log("initial   format string:"+JSON.stringify(fmt));
  if (DEBUG) console.log("\n\nwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww\n\n");
    return this.WouldPack(fmt, new Array(this.CalcLength(fmt)), 0, values);
  //if (DEBUG) console.log("\n\nmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm\n\n");
   // return this.PackTo(fmt, new Array(this.CalcLength(fmt)), 0, values);
  };

  /**
   * @param {string} fmt
   * @param {(Node|NodeList|null|string)} values
   * @return {?}
   */
  	// Pack the supplied values into a new octet array, according to the fmt string
  m.oldPack = function(fmt, values) {
  if (DEBUG) console.log("\n\n------------------------------------------------------------------------------------------------------------\n\n");
  if (DEBUG) console.log("initial unpacked values:"+JSON.stringify(values));
  if (DEBUG) console.log("initial   format string:"+JSON.stringify(fmt));
    return this.PackTo(fmt, new Array(this.CalcLength(fmt)), 0, values);
  };
  
  /**
   * @param {string} fmt
   * @return {?}
   */
  	// Determine the number of bytes represented by the format string
  m.CalcLength = function(fmt) {

    /** @type {!RegExp} */
    var re = new RegExp(this._sPattern, "g");
    var m;
    /** @type {number} */
    var value = 0;
    for (; m = re.exec(fmt);) {
      /** @type {number} */
      value = value + (m[1] == undefined || m[1] == "" ? 1 : parseInt(m[1])) * this._lenLut[m[2]];
    }
  if (DEBUG) console.log("number of bytes in format string?: "+value+"\n");
    return value;
  };
}
exports.jspack = new JSPack;

