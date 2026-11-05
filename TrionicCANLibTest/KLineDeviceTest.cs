using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using TrionicCANLib.KWP;

namespace TrionicCANLibTest
{
    [TestClass]
    public class KLineDeviceTest
    {
        // a K-line with DICE and a T7 behind it, frames as hex: the header and payload a transport is handed
        class FakeKLine : KLineDevice
        {
            public readonly List<string> Sent = new List<string>();
            public readonly List<bool> Timing = new List<bool>();
            public Func<string, string[]> Answer; // request frame -> reply payloads
            private readonly Queue<string> m_replies = new Queue<string>();

            public override bool open() { return true; }
            public override bool close() { return true; }
            public override bool isOpen() { return true; }
            protected override void setTiming(bool a_fast) { Timing.Add(a_fast); }
            protected override byte[] fastInit(byte[] a_frame) { return send(a_frame) ? receive() : null; }

            protected override bool send(byte[] a_frame)
            {
                string frame = Convert.ToHexString(a_frame);
                Sent.Add(frame);
                foreach (string reply in Answer(frame))
                    m_replies.Enqueue(reply);
                return true;
            }

            protected override byte[] receive()
            {
                return m_replies.Count > 0 ? Convert.FromHexString(m_replies.Dequeue()) : null;
            }
        }

        static string[] Car(string a_frame)
        {
            switch (a_frame)
            {
                case "8111F181": return new[] { "C1EF8F" };
                case "8711F183030002001400": return new[] { "C3" };
                case "8141F181": return new[] { "C1EF8F" };
                case "8541F12300005F01": return new[] { "6380" };
                case "8641F13D00005F0188": return new[] { "7D" };
                case "8011F1082CF00300F5010203": return new[] { "7F2C78", "7F2C78", "6CF0" };
                default: return new string[0];
            }
        }

        [TestMethod]
        public void SessionSetsTimingAndDiceFastKLine()
        {
            var line = new FakeKLine { Answer = Car };
            Assert.IsTrue(line.startSession());
            CollectionAssert.AreEqual(new[] { "8111F181", "8141F181", "8541F12300005F01", "8641F13D00005F0188", "8711F183030002001400" }, line.Sent,
                "fast init, DICE's $5F bit 3 set keeping bit 7 at the default P3, then P3min 0 from the T7");
            CollectionAssert.AreEqual(new[] { false, true }, line.Timing);

            line.Sent.Clear();
            Assert.IsTrue(line.startSession(), "a session that just answered is kept");
            Assert.AreEqual(0, line.Sent.Count);
        }

        [TestMethod]
        public void RequestOverSevenBytesHasLengthByteAndWaitsOutPending()
        {
            var line = new FakeKLine { Answer = Car };
            Assert.IsTrue(line.startSession());
            line.Sent.Clear();

            Assert.AreEqual(RequestResult.NoError, line.sendRequest(new KWPRequest(0x2C, 0xF0, 0x03, new byte[] { 0x00, 0xF5, 0x01, 0x02, 0x03 }), out KWPReply reply));
            CollectionAssert.AreEqual(new[] { "8011F1082CF00300F5010203" }, line.Sent, "8 bytes: a length byte, not 0x88");
            Assert.AreEqual(0x6C, reply.getMode(), "responsePending is not the reply");
            Assert.AreEqual(2, reply.getLength());

            Assert.AreEqual(RequestResult.Timeout, line.sendRequest(new KWPRequest(0x21, 0xF0), out reply));
        }
    }
}
