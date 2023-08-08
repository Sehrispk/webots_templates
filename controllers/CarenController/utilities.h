#ifndef TCP_CONTROLLER_UTILITIES
#define TCP_CONTROLLER_UTILITIES

void split(const std::string &str,
           const std::string &separator,
           std::vector <std::string> &parts)
{
    parts.clear();

    size_t last_index = 0;
    size_t index = str.find(separator);
    while (index != std::string::npos)
    {
        std::string chunk = str.substr(last_index, index - last_index);
        parts.push_back(chunk);
        last_index = index + separator.size();
        index = str.find(separator, last_index);
    }
    std::string chunk = str.substr(last_index, index - last_index);
    parts.push_back(chunk);
}

/* This is the basic CRC-32 calculation with some optimization but no
table lookup. The the byte reversal is avoided by shifting the crc reg
right instead of left and by using a reversed 32-bit word to represent
the polynomial.
   When compiled to Cyclops with GCC, this function executes in 8 + 72n
instructions, where n is the number of bytes in the input message. It
should be doable in 4 + 61n instructions.
   If the inner loop is strung out (approx. 5*8 = 40 instructions),
it would take about 6 + 46n instructions. */

unsigned int generateChecksum(const char *message, int message_length)
{
    int j;
    unsigned int byte, crc, mask;

    crc = 0xFFFFFFFF;
    for (int i = 0; i < message_length; i++)
    {
        byte = message[i];            // Get next byte.
        crc = crc ^ byte;
        for (j = 7; j >= 0; j--)
        {    // Do eight times.
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
        i = i + 1;
    }
    return ~crc;
}

#endif // TCP_CONTROLLER_UTILITIES