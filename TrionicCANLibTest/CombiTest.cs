using System;
using System.Collections.Generic;
using System.Text;
using Combi;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace TrionicCANLibTest
{
    [TestClass]
    public class CombiTest
    {
        [TestMethod]
        public void BuildsTxFramePacketLikeGocan()
        {
            // vector from gocan adapters/combi TestCombiEncode: ID 0x220, data 3F 81
            caCombiAdapter.caCANFrame frame = new caCombiAdapter.caCANFrame { id = 0x220, length = 2, data = 0x813F };
            byte[] pkt = caCombiAdapter.BuildPacket(0x83, caCombiAdapter.EncodeFrame(frame), caCombiAdapter.term_ack);

            Assert.AreEqual("83000F200200003F8100000000000002000000", Convert.ToHexString(pkt));
            Assert.AreEqual("8000010100", Convert.ToHexString(caCombiAdapter.BuildPacket(0x80, new byte[] { 1 }, 0)));
            Assert.AreEqual("8A0000FF", Convert.ToHexString(caCombiAdapter.BuildPacket(0x8a, null, caCombiAdapter.term_nak)));
        }

        [TestMethod]
        public void ParsesPacketsAcrossTransfersAndResyncs()
        {
            var got = new List<string>();
            var parser = new caCombiAdapter.caPacketParser((cmd, data, term) =>
                got.Add(string.Format("{0:X2}:{1}:{2:X2}", cmd, Convert.ToHexString(data), term)));

            byte[] rx = Convert.FromHexString(
                "55" +                                      // garbage before a packet, skipped
                "82000F" + "E8070000" + "0102030405060708" + "08" + "00" + "00" + "00" +  // CAN frame 0x7E8
                "800000" + "00" +                           // zero length ack
                "830000FF" +                                // TX NAK
                "200002" + "0301" + "00");                  // firmware 1.3

            // split so a header and its terminator land in different transfers
            int[] cuts = { 1, 5, 19, 22, 23, rx.Length };
            int start = 0;
            foreach (int cut in cuts)
            {
                byte[] chunk = new byte[cut - start];
                Array.Copy(rx, start, chunk, 0, chunk.Length);
                parser.Feed(chunk, chunk.Length);
                start = cut;
            }

            CollectionAssert.AreEqual(new[]
            {
                "82:E80700000102030405060708080000:00",
                "80::00",
                "83::FF",
                "20:0301:00",
            }, got);

            caCombiAdapter.caCANFrame frame = caCombiAdapter.DecodeFrame(Convert.FromHexString("E80700000102030405060708080000"));
            Assert.AreEqual(0x7E8u, frame.id);
            Assert.AreEqual(0x0807060504030201ul, frame.data);
            Assert.AreEqual((byte)8, frame.length);
        }

        [TestMethod]
        public void FlashChecksumIsCrc32()
        {
            Assert.AreEqual(0xCBF43926u, ~caCombiAdapter.AddCrc32(0xffffffff, Encoding.ASCII.GetBytes("123456789")));

            // running over blocks gives the same result as one pass
            byte[] all = Encoding.ASCII.GetBytes("123456789");
            uint crc = caCombiAdapter.AddCrc32(0xffffffff, all[..4]);
            Assert.AreEqual(0xCBF43926u, ~caCombiAdapter.AddCrc32(crc, all[4..]));
        }
    }
}
