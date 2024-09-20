using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Ball_Ballancer_CS.Class
{
    internal class UTF8
    {
        List<byte> RemainBytes = new List<byte>();

        bool IsUTF8(byte _byte)
        {
            if ((_byte & 0xE0) == 0xE0) return true;
            return false;

        }

        public String AddBytes(List<byte> _bytes)
        {
            RemainBytes.AddRange(_bytes);

            if (this.RemainBytes.Count >= 2 && IsUTF8(this.RemainBytes[this.RemainBytes.Count - 2]))
            {
                String s = System.Text.Encoding.UTF8.GetString(RemainBytes.ToArray(), 0, this.RemainBytes.Count - 2);
                RemainBytes.RemoveRange(0, this.RemainBytes.Count - 2);
                return s;
            }
            else if (this.RemainBytes.Count >= 1 && IsUTF8(this.RemainBytes[this.RemainBytes.Count - 1]))
            {
                String s = System.Text.Encoding.UTF8.GetString(RemainBytes.ToArray(), 0, this.RemainBytes.Count - 1);
                RemainBytes.RemoveRange(0, this.RemainBytes.Count - 1);
                return s;
            }
            else
            {
                String s = System.Text.Encoding.UTF8.GetString(RemainBytes.ToArray(), 0, this.RemainBytes.Count);
                RemainBytes.Clear();
                return s;
            }

        }

    }
}
