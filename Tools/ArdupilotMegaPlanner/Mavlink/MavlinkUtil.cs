using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using log4net;

namespace ArdupilotMega.Mavlink
{
    /// <summary>
    /// Static methods and helpers for creation and manipulation of Mavlink packets
    /// </summary>
    public static class MavlinkUtil
    {
        private static readonly ILog log =
            LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        /// <summary>
        /// Create a new mavlink packet object from a byte array as recieved over mavlink
        /// Endianess will be detetected using packet inspection
        /// </summary>
        /// <typeparam name="TMavlinkPacket">The type of mavlink packet to create</typeparam>
        /// <param name="bytearray">The bytes of the mavlink packet</param>
        /// <param name="startoffset">The position in the byte array where the packet starts</param>
        /// <returns>The newly created mavlink packet</returns>
        public static TMavlinkPacket ByteArrayToStructure<TMavlinkPacket>(this byte[] bytearray, int startoffset) where TMavlinkPacket : struct
        {
            object newPacket = new TMavlinkPacket();
            ByteArrayToStructure(bytearray, ref newPacket, startoffset);
            return (TMavlinkPacket)newPacket;
        }

        public static TMavlinkPacket ByteArrayToStructureBigEndian<TMavlinkPacket>(this byte[] bytearray, int startoffset) where TMavlinkPacket : struct
        {
            object newPacket = new TMavlinkPacket();
            ByteArrayToStructureEndian(bytearray, ref newPacket, startoffset);
            return (TMavlinkPacket)newPacket;
        }

        public static void ByteArrayToStructure(byte[] bytearray, ref object obj, int startoffset)
        {
            if (bytearray[0] == 'U')
            {
                ByteArrayToStructureEndian(bytearray, ref obj, startoffset);
            }
            else
            {
                int len = Marshal.SizeOf(obj);

                IntPtr i = Marshal.AllocHGlobal(len);

                // create structure from ptr
                obj = Marshal.PtrToStructure(i, obj.GetType());

                try
                {
                    // copy byte array to ptr
                    Marshal.Copy(bytearray, startoffset, i, len);
                }
                catch (Exception ex)
                {
                    log.Error("ByteArrayToStructure FAIL", ex);
                }

                obj = Marshal.PtrToStructure(i, obj.GetType());

                Marshal.FreeHGlobal(i);
            }
        }

        public static void ByteArrayToStructureEndian(byte[] bytearray, ref object obj, int startoffset)
        {
            int len = Marshal.SizeOf(obj);
            IntPtr i = Marshal.AllocHGlobal(len);
            byte[] temparray = (byte[])bytearray.Clone();

            // create structure from ptr
            obj = Marshal.PtrToStructure(i, obj.GetType());

            // do endian swap
            object thisBoxed = obj;
            Type test = thisBoxed.GetType();

            int reversestartoffset = startoffset;

            // Enumerate each structure field using reflection.
            foreach (var field in test.GetFields())
            {
                // field.Name has the field's name.
                object fieldValue = field.GetValue(thisBoxed); // Get value

                // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
                TypeCode typeCode = Type.GetTypeCode(fieldValue.GetType());

                if (typeCode != TypeCode.Object)
                {
                    Array.Reverse(temparray, reversestartoffset, Marshal.SizeOf(fieldValue));
                    reversestartoffset += Marshal.SizeOf(fieldValue);
                }
                else
                {
                    reversestartoffset += ((byte[])fieldValue).Length;
                }

            }

            try
            {
                // copy byte array to ptr
                Marshal.Copy(temparray, startoffset, i, len);
            }
            catch (Exception ex)
            {
                log.Error("ByteArrayToStructure FAIL", ex);
            }

            obj = Marshal.PtrToStructure(i, obj.GetType());

            Marshal.FreeHGlobal(i);

        }

        /// <summary>
        /// Convert a struct to an array of bytes, struct fields being reperesented in 
        /// little endian (LSB first)
        /// </summary>
        /// <remarks>Note - assumes little endian host order</remarks>
        public static byte[] StructureToByteArray(object obj)
        {
            int len = Marshal.SizeOf(obj);
            byte[] arr = new byte[len];
            IntPtr ptr = Marshal.AllocHGlobal(len);
            Marshal.StructureToPtr(obj, ptr, true);
            Marshal.Copy(ptr, arr, 0, len);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }

       /// <summary>
       /// Convert a struct to an array of bytes, struct fields being reperesented in 
       /// big endian (MSB first)
       /// </summary>
        public static byte[] StructureToByteArrayBigEndian(params object[] list)
        {
            // The copy is made becuase SetValue won't work on a struct.
            // Boxing was used because SetValue works on classes/objects.
            // Unfortunately, it results in 2 copy operations.
            object thisBoxed = list[0]; // Why make a copy?
            Type test = thisBoxed.GetType();

            int offset = 0;
            byte[] data = new byte[Marshal.SizeOf(thisBoxed)];

            object fieldValue;
            TypeCode typeCode;

            byte[] temp;

            // Enumerate each structure field using reflection.
            foreach (var field in test.GetFields())
            {
                // field.Name has the field's name.

                fieldValue = field.GetValue(thisBoxed); // Get value

                // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
                typeCode = Type.GetTypeCode(fieldValue.GetType());

                switch (typeCode)
                {
                    case TypeCode.Single: // float
                        {
                            temp = BitConverter.GetBytes((Single)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Single));
                            break;
                        }
                    case TypeCode.Int32:
                        {
                            temp = BitConverter.GetBytes((Int32)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Int32));
                            break;
                        }
                    case TypeCode.UInt32:
                        {
                            temp = BitConverter.GetBytes((UInt32)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(UInt32));
                            break;
                        }
                    case TypeCode.Int16:
                        {
                            temp = BitConverter.GetBytes((Int16)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Int16));
                            break;
                        }
                    case TypeCode.UInt16:
                        {
                            temp = BitConverter.GetBytes((UInt16)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(UInt16));
                            break;
                        }
                    case TypeCode.Int64:
                        {
                            temp = BitConverter.GetBytes((Int64)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Int64));
                            break;
                        }
                    case TypeCode.UInt64:
                        {
                            temp = BitConverter.GetBytes((UInt64)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(UInt64));
                            break;
                        }
                    case TypeCode.Double:
                        {
                            temp = BitConverter.GetBytes((Double)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Double));
                            break;
                        }
                    case TypeCode.Byte:
                        {
                            data[offset] = (Byte)fieldValue;
                            break;
                        }
                    default:
                        {
                            //System.Diagnostics.Debug.Fail("No conversion provided for this type : " + typeCode.ToString());
                            break;
                        }
                }; // switch
                if (typeCode == TypeCode.Object)
                {
                    int length = ((byte[])fieldValue).Length;
                    Array.Copy(((byte[])fieldValue), 0, data, offset, length);
                    offset += length;
                }
                else
                {
                    offset += Marshal.SizeOf(fieldValue);
                }
            } // foreach

            return data;
        } // Swap
    }
}
