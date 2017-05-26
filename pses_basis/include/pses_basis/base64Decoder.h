#include<vector>
#include<string>
//#include <bitset>

// from azawadzki, https://stackoverflow.com/questions/180947/base64-decode-snippet-in-c

static std::vector<unsigned char> base64_decode(const std::string &in) {

    std::string out;
    std::string in_padded;

    // Add padding
    if (in_padded.length() % 4 == 2) {
      in_padded = in + "==";
    }

    else if (in_padded.length() % 4 == 3) {
      in_padded = in + "=";
    }

    else {
      in_padded = in;
    }

    std::vector<int> T(256,-1);
    for (int i=0; i<64; i++) T["ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[i]] = i;

    int val=0, valb=-8;
    for (unsigned char c : in_padded) {
        if (T[c] == -1) break;
        val = (val<<6) + T[c];
        valb += 6;
        if (valb>=0) {
            out.push_back(char((val>>valb)&0xFF));
            valb-=8;
        }
    }
    return std::vector<unsigned char>(out.begin(), out.end());
}

static int getValueAt(std::vector<unsigned char>& bytes, int startByteIndex, int endByteIndex) {

  int offset = endByteIndex - startByteIndex;
  int out = 0;

  for(int i = endByteIndex; i >= startByteIndex; i--) {
      out = out | (0x00000000 | bytes[i] << 8*offset);
      offset -= 1;
  }
  return out;
}

/*int main() {
  std::vector<unsigned char> bytes = base64_decode("Afb/");
  //std::cout << (int)(bytes[2] << 8 | bytes[1]) << std::endl;
  getValueAt(bytes, 1, 2);
}*/
