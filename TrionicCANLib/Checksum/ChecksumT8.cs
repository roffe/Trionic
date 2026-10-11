using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using NLog;
using TrionicCANLib.Firmware;

namespace TrionicCANLib.Checksum
{
    public class ChecksumT8
    {
        private readonly static Logger logger = LogManager.GetCurrentClassLogger();

        /// <summary>
        /// Checks both layers and, when told to, corrects them. layer2ByteSum is the file's layer-2 method (Layer2ByteSum, read
        /// before editing); without it the stored value decides, which an edit can make it guess wrong.
        /// </summary>
        public static ChecksumResult VerifyChecksum(string filename, bool autocorrect, ChecksumDelegate.ChecksumUpdate delegateShouldUpdate, bool? layer2ByteSum = null)
        {
            int checksumAreaOffset = GetChecksumAreaOffset(filename);
            if (checksumAreaOffset > FileT8.Length)
            {
                return ChecksumResult.InvalidFileLength;
            }

            logger.Debug("Checksum area offset: " + checksumAreaOffset.ToString("X8"));
            byte[] hash = CalculateLayer1ChecksumMD5(filename, checksumAreaOffset);
            // compare hash to bytes after checksum areaoffset
            byte[] layer1checksuminfile = FileTools.readdatafromfile(filename, checksumAreaOffset + 2, 16);
            if (!CompareByteArray(hash, layer1checksuminfile))
            {
                if (autocorrect)
                {
                    if (!FileTools.savedatatobinary(checksumAreaOffset + 2, 16, hash, filename))
                    {
                        return ChecksumResult.UpdateFailed;
                    }
                }
                else
                {
                    string filechecksum = string.Empty;
                    string realchecksum = string.Empty;
                    for (int i = 0; i < layer1checksuminfile.Length; i++)
                    {
                        filechecksum += layer1checksuminfile[i].ToString("X2") + " ";
                        realchecksum += hash[i].ToString("X2") + " ";
                    }
                    if (delegateShouldUpdate("Checksum validation Layer 1", filechecksum, realchecksum))
                    {
                        if (!FileTools.savedatatobinary(checksumAreaOffset + 2, 16, hash, filename))
                        {
                            return ChecksumResult.UpdateFailed;
                        }
                    }
                    else
                    {
                        return ChecksumResult.Layer1Failed;
                    }
                }
            }

            return CalculateLayer2Checksum(filename, checksumAreaOffset, autocorrect, delegateShouldUpdate, layer2ByteSum);
        }

        public static int GetChecksumAreaOffset(string filename)
        {
            int retval = 0;
            if (filename == "") return retval;
            FileStream fsread = new FileStream(filename, FileMode.Open, FileAccess.Read);
            using (BinaryReader br = new BinaryReader(fsread))
            {
                fsread.Seek(0x20140, SeekOrigin.Begin);
                retval = (int)br.ReadByte() * 256 * 256 * 256;
                retval += (int)br.ReadByte() * 256 * 256;
                retval += (int)br.ReadByte() * 256;
                retval += (int)br.ReadByte();
            }
            fsread.Close();
            return retval;
        }

        static private byte[] CalculateLayer1ChecksumMD5(string filename, int OffsetLayer1)
        {
            /*
            1.	calculate checksum pointer
            2.	Checksum is 2 level based on Message Digest 5 algorithm
            3.	Pointer = @ 0x20140 and is a 4 byte pointer
            4.	Use MD5 to make 16 bytes digest from any string
            5.	name checksum pointer CHPTR
            6.	checksum area 1 ranges from 20000h to CHPTR – 20000h- 1
            7.	Create an MD5 hash from this string (20000h – (CHPTR – 20000h – 1))
                a.	MD5Init(Context)
                b.	MD5Update(Context, buffer, size)
                c.	MD5Final(Context, Md5Seed)
                d.	sMd5Seed = MD5Print(Md5Seed)
                e.	sMd5Seed = 16 bytes hex, so 32 bytes.
                f.	Now crypt sMd5Seed: xor every byte with 21h, then substract D6h (minus)
                g.	These 16 bytes are from CHPTR + 2 in the bin!!!! This is checksum level 1 !!
                         * */
            System.Security.Cryptography.MD5 md5 = System.Security.Cryptography.MD5.Create();

            int len = OffsetLayer1 - 0x20000;//- 1;
            md5.Initialize();
            int end = 0x20000 + len;
            logger.Debug("Calculating from 0x20000 upto " + end.ToString("X8"));

            byte[] data = FileTools.readdatafromfile(filename, 0x20000, len);
            byte[] hash = md5.ComputeHash(data);
            byte[] finalhash = new byte[hash.Length];

            for (int i = 0; i < hash.Length; i++)
            {
                byte bcalc = hash[i];
                bcalc ^= 0x21;
                bcalc -= 0xD6;
                finalhash[i] = bcalc;
            }

            return finalhash;

        }

        static private bool CompareByteArray(byte[] arr1, byte[] arr2)
        {
            bool retval = true;
            if (arr1.Length != arr2.Length) retval = false;
            else
            {
                for (int t = 0; t < arr1.Length; t++)
                {
                    if (arr1[t] != arr2[t]) retval = false;
                }
            }
            return retval;
        }

        /// <summary>
        /// Whether the file's layer-2 checksum is the byte sum (true) or the dword sum (false), null when it is neither. Read it
        /// while the file verifies and pass it to VerifyChecksum after editing: an edit that carries the byte sum over a 1 MiB
        /// boundary changes its top 12 bits, and the stored value's top 12 bits were all that told the methods apart.
        /// </summary>
        public static bool? Layer2ByteSum(string filename)
        {
            byte[] complete_file = File.ReadAllBytes(filename);
            if (complete_file.Length != FileT8.Length) return null;
            int offset = complete_file[0x20140] << 24 | complete_file[0x20141] << 16 | complete_file[0x20142] << 8 | complete_file[0x20143];
            if (!ReadLayer2(complete_file, offset, out _, out uint sum0, out uint byteSum, out uint dwordSum)) return null;
            if (sum0 == byteSum) return true;
            if (sum0 == dwordSum) return false;
            return null;
        }

        /// <summary>
        /// Finds the layer-2 record in the checksum area: its index in the coded buffer, the stored sum, and the byte sum and the
        /// dword sum over the range it names. False when there is none.
        /// </summary>
        private static bool ReadLayer2(byte[] complete_file, int OffsetLayer2, out int index, out uint sum0, out uint byteSum, out uint dwordSum)
        {
            sum0 = byteSum = dwordSum = 0;
            /*
            Get 0x100 byte buffer from CHPTR – CHPTR + 0xFF
            Because level 1 is in that area level1 must be correct first
            Prepare coded_buffer (0x100 buffer from chptr) with loop: coded_buffer(x) = (buffer (x) + 0xD6) xor 0x21 
            (add 0xd6 to every byte of buffer, then xor it by 0x21)
            [ 1 indexed, not 0 indexed ]
            So, 0x101 byte buffer with first byte ignored (convention)
                         * */
            byte[] coded_buffer = new byte[0x100];
            for (int i = 0; i < coded_buffer.Length && OffsetLayer2 >= 0 && OffsetLayer2 + i < complete_file.Length; i++)
            {
                byte b = complete_file[OffsetLayer2 + i];
                b += 0xD6;
                b ^= 0x21;
                coded_buffer[i] = b;
            }

            // the record is 0x11 bytes: FB sum, FC end of the range, FD its start
            for (index = 0; index + 0x10 < coded_buffer.Length; index++)
            {
                if ((coded_buffer[index] != 0xFB) || (coded_buffer[index + 6] != 0xFC) || (coded_buffer[index + 0x0C] != 0xFD))
                {
                    continue;
                }
                sum0 = ((uint)coded_buffer[index + 1] * 0x01000000 + (uint)coded_buffer[index + 2] * 0x010000 + (uint)coded_buffer[index + 3] * 0x100 + (uint)coded_buffer[index + 4]);
                uint matrix_dimension = (uint)coded_buffer[index + 7] * 0x01000000 + (uint)coded_buffer[index + 8] * 0x010000 + (uint)coded_buffer[index + 9] * 0x100 + (uint)coded_buffer[index + 10];
                uint partial_address = (uint)coded_buffer[index + 0x0d] * 0x01000000 + (uint)coded_buffer[index + 0x0e] * 0x010000 + (uint)coded_buffer[index + 0x0F] * 0x100 + (uint)coded_buffer[index + 0x10];
                if (matrix_dimension < 0x020000 || matrix_dimension > complete_file.Length || partial_address > matrix_dimension)
                {
                    continue;
                }
                byteSum = 0;
                uint x = partial_address;
                while (x < (matrix_dimension - 4))
                {
                    byteSum = byteSum + (uint)complete_file[x];
                    x++;
                }
                byteSum = byteSum + (uint)complete_file[matrix_dimension - 1];
                dwordSum = 0;
                x = partial_address;
                while (x < (matrix_dimension - 4))
                {
                    dwordSum = dwordSum + (uint)complete_file[x] * 0x01000000 + (uint)complete_file[x + 1] * 0x10000 + (uint)complete_file[x + 2] * 0x100 + (uint)complete_file[x + 3];
                    x = x + 4;
                }
                return true;
            }
            return false;
        }

        static private ChecksumResult CalculateLayer2Checksum(string filename, int OffsetLayer2, bool autocorrect, ChecksumDelegate.ChecksumUpdate delegateShouldUpdate, bool? layer2ByteSum)
        {
            byte[] complete_file = FileTools.readdatafromfile(filename, 0, 0x100000);
            if (!ReadLayer2(complete_file, OffsetLayer2, out int index, out uint sum0, out uint checksum0, out uint checksum1))
            {
                logger.Debug("Layer 2 checksum could not be calculated [ file incompatible ]");
                return ChecksumResult.Layer2Failed;
            }
            // the byte sum or the dword sum: the caller's, else the one the stored value is, else the guess that the byte sum still
            // has the stored value's top 12 bits
            bool byteSum = layer2ByteSum ?? (sum0 == checksum0 || (sum0 != checksum1 && (checksum0 & 0xFFF00000) == (sum0 & 0xFFF00000)));
            if (!byteSum)
            {
                checksum0 = checksum1;
            }
            if (checksum0 == sum0)
            {
                return ChecksumResult.Ok;
            }
            logger.Debug("Layer 2 checksum was invalid, should be updated!");
            if (autocorrect)
            {
                return UpdateLayer2(filename, OffsetLayer2, checksum0, index);
            }
            string filechecksum = sum0.ToString("X8");
            string realchecksum = checksum0.ToString("X8");
            if (delegateShouldUpdate("Checksum validation Layer 2", filechecksum, realchecksum))
            {
                return UpdateLayer2(filename, OffsetLayer2, checksum0, index);
            }
            return ChecksumResult.Layer2Failed;
        }

        private static ChecksumResult UpdateLayer2(string filename, int OffsetLayer2, uint checksum0, int index)
        {
            ChecksumResult result;
            byte[] checksum_to_file = new byte[4];
            checksum_to_file[0] = Convert.ToByte((checksum0 >> 24) & 0x000000FF);
            checksum_to_file[1] = Convert.ToByte((checksum0 >> 16) & 0x000000FF);
            checksum_to_file[2] = Convert.ToByte((checksum0 >> 8) & 0x000000FF);
            checksum_to_file[3] = Convert.ToByte((checksum0) & 0x000000FF);
            checksum_to_file[0] = Convert.ToByte(((checksum_to_file[0] ^ 0x21) - (byte)0xD6) & 0x000000FF);
            checksum_to_file[1] = Convert.ToByte(((checksum_to_file[1] ^ 0x21) - (byte)0xD6) & 0x000000FF);
            checksum_to_file[2] = Convert.ToByte(((checksum_to_file[2] ^ 0x21) - (byte)0xD6) & 0x000000FF);
            checksum_to_file[3] = Convert.ToByte(((checksum_to_file[3] ^ 0x21) - (byte)0xD6) & 0x000000FF);
            if (FileTools.savedatatobinary(index + OffsetLayer2 + 1, 4, checksum_to_file, filename))
            {
                result = ChecksumResult.Ok;
            }
            else
            {
                result = ChecksumResult.UpdateFailed;
            }
            return result;
        }
    }
}
