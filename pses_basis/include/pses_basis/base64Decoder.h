#include <vector>
#include <string>

/*** from azawadzki,
* https://stackoverflow.com/questions/180947/base64-decode-snippet-in-c
*/

inline std::vector<unsigned char> base64_to_binary(const std::string& in)
{

  std::string out;
  std::string in_padded;

  // Add padding
  if (in_padded.length() % 4 == 2)
  {
    in_padded = in + "==";
  }

  else if (in_padded.length() % 4 == 3)
  {
    in_padded = in + "=";
  }

  else
  {
    in_padded = in;
  }

  std::vector<int> T(256, -1);
  for (int i = 0; i < 64; i++)
    T["ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[i]] =
        i;

  int val = 0, valb = -8;
  for (unsigned char c : in_padded)
  {
    if (T[c] == -1)
      break;
    val = (val << 6) + T[c];
    valb += 6;
    if (valb >= 0)
    {
      out.push_back(char((val >> valb) & 0xFF));
      valb -= 8;
    }
  }
  return std::vector<unsigned char>(out.begin(), out.end());
}

inline int getValueAt(std::vector<unsigned char>& bytes,
                      const unsigned int startByteIndex,
                      const unsigned int endByteIndex)
{

  int offset = endByteIndex - startByteIndex;
  int out = 0;

  for (int i = endByteIndex; i >= startByteIndex; i--)
  {
    out = out | (0x00000000 | bytes[i] << 8 * offset);
    offset -= 1;
  }
  return out;
}

inline int convertValue(const int value, const unsigned int size, bool isSigned)
{
  int val = value;
  int mask = 1 << size - 1;
  int sign = val & mask;
  sign = sign >> size - 1;
  if (sign == 1 && isSigned)
  {
    int offset = 32 - size;
    int sign_extension = 1;

    for (int i = 1; i < offset; i++)
    {
      sign_extension = (sign_extension << 1) | 1;
    }

    sign_extension = sign_extension << size;
    val |= sign_extension;
  }
  return val;
}

/** Function decodes a Base64 string and returns the decoded string (or only
 *parts of it) as an int32.
 * This Fuction decodes a Base64 string and returns a specific byte or multiple
 *bytes at the given indices,
 * where this information is converted as a desired datatype and returned as
 *int32.
 *
 * Example for decoding a Base64 string, getting the second byte, reading this
 *value as an int16:
 * base64_decode("Afb/", 1, 2, 16, true).
 * @param in input Base64 string.
 * @param startByteIndex index number (starting with zero) of the first byte of
 *the value that has to be read.
 * @param endByteIndex index number (starting with zero) of the last byte of the
 *value that has to be read.
 * @param size size in bits of the value that is to be read from the decoded
 *string.
 * @param isSigned whether the value from the decoded string is to be read as
 *signed or unsigned.
 * @return Decoded information as an int32.
*/
inline int base64_decode(const std::string& in,
                         const unsigned int startByteIndex,
                         const unsigned int endByteIndex,
                         const unsigned int size, bool isSigned)
{
  std::vector<unsigned char> bytes = base64_to_binary(in);
  int value = getValueAt(bytes);
  return convertValue(value, size, isSigned);
}
