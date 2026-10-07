
/*
 * Protocol2019Handler.cpp
 *
 * Created on: Apr 25, 2024
 * Author: Varalakshmi
 */

#include "Protocol2019Handler.h"

Protocol2019Handler protocol2019Handler;

namespace {
bool isAllDigits(const String& value) {
    if (value.length() == 0) {
        return false;
    }

    for (int i = 0; i < value.length(); ++i) {
        if (!isDigit(value.charAt(i))) {
            return false;
        }
    }

    return true;
}

bool parseMagazine2Command(const String& pair, long& mhValue, long& mh2SampleCountValue) {
    if (pair.equals("MH2")) {
        mhValue = 2;
        return true;
    }

    if (pair.startsWith("MH2=")) {
        String countStr = pair.substring(4);
        countStr.trim();
        if (!isAllDigits(countStr)) {
            return false;
        }
        mh2SampleCountValue = countStr.toInt();
        return true;
    }

    if (!pair.startsWith("MH2") || pair.length() <= 3) {
        return false;
    }

    String suffix = pair.substring(3);
    suffix.trim();

    if (suffix.equals("11")) {
        mhValue = 211;
        return true;
    }
    if (suffix.equals("12")) {
        mhValue = 212;
        return true;
    }
    if (suffix.equals("21")) {
        mhValue = 221;
        return true;
    }
    if (suffix.equals("22")) {
        mhValue = 222;
        return true;
    }
    if (suffix.equals("4")) {
        mhValue = 24;
        return true;
    }
    if (suffix.equals("5")) {
        mhValue = 25;
        return true;
    }
    if (suffix.equals("6")) {
        mhValue = 26;
        return true;
    }
    if (suffix.equals("7")) {
        mhValue = 27;
        return true;
    }

    if (!isAllDigits(suffix)) {
        return false;
    }

    mh2SampleCountValue = suffix.toInt();
    return true;
}

bool parseMagazine2PulseCommand(const String& pair, long& mhp2SampleCountValue) {
    if (pair.startsWith("MHP2=")) {
        String countStr = pair.substring(5);
        countStr.trim();
        if (!isAllDigits(countStr)) {
            return false;
        }
        mhp2SampleCountValue = countStr.toInt();
        return true;
    }

    if (!pair.startsWith("MHP2") || pair.length() <= 4) {
        return false;
    }

    String suffix = pair.substring(4);
    suffix.trim();
    if (!isAllDigits(suffix)) {
        return false;
    }

    mhp2SampleCountValue = suffix.toInt();
    return true;
}
}

Protocol2019Handler::Protocol2019Handler()
{
    resetValues();
}

Protocol2019Handler::~Protocol2019Handler()
{
    // Destructor
}

bool Protocol2019Handler::parsePacket(ByteArrayHandler &packet)
{
    // Convert byte array to string
    String packetStr = "";
    for (int i = 0; i < packet.getSize(); i++)
    {
        packetStr += (char)packet.getByte(i);
    }

    // Split the string by commas
    int startIndex = 0;
    int commaIndex = packetStr.indexOf(',');

    while (commaIndex != -1 || startIndex < packetStr.length())
    {
        // Extract key-value pair
        String pair;
        if (commaIndex == -1)
        {
            // Last pair
            pair = packetStr.substring(startIndex);
            startIndex = packetStr.length();
        }
        else
        {
            pair = packetStr.substring(startIndex, commaIndex);
            startIndex = commaIndex + 1;
            commaIndex = packetStr.indexOf(',', startIndex);
        }

        if (parseMagazine2PulseCommand(pair, mhp2SampleCountValue)) {
            continue;
        }
        if (pair.equals("MH1")) {
            mhValue = 1;
            continue;
        }
        if (parseMagazine2Command(pair, mhValue, mh2SampleCountValue)) {
            continue;
        }
        if (pair.equals("CAL1")) {
            cal1Value = 1;
            continue;
        }
        if (pair.equals("CAL2")) {
            cal2Value = 1;
            continue;
        }

        // M14 / M15 / M17 / M18 motor commands — keys contain digits, so the
        // generic letters-then-digits key parser below would chop them at the
        // first numeric character. Match each suffix-then-base explicitly.
        // Longer suffixes first (M14GP before M14R before M14, etc.).
        {
            String tp = pair; tp.trim();
            #define _MATCH_MOTOR_CMD(PREFIX, FIELD)                  \
                if (tp.startsWith(PREFIX)) {                         \
                    String tail = tp.substring(strlen(PREFIX));      \
                    tail.trim();                                     \
                    FIELD = (tail.length() == 0) ? 1 : tail.toInt(); \
                    continue;                                        \
                }
            // M14  (M14ML must come before M14M — startsWith would otherwise mis-match)
            _MATCH_MOTOR_CMD("M14ML", m14mlValue)   // dispense by mL × 1000 (µL)
            _MATCH_MOTOR_CMD("M14GP", m14gpValue)
            _MATCH_MOTOR_CMD("M14R",  m14rValue)
            _MATCH_MOTOR_CMD("M14I",  m14iValue)
            _MATCH_MOTOR_CMD("M14M",  m14mValue)
            _MATCH_MOTOR_CMD("M14S",  m14sValue)
            _MATCH_MOTOR_CMD("M14A",  m14aValue)
            _MATCH_MOTOR_CMD("M14",   m14Value)
            // M15  (M15ML must come before M15M — startsWith would otherwise mis-match)
            _MATCH_MOTOR_CMD("M15ML", m15mlValue)   // dispense by mL × 1000 (µL)
            _MATCH_MOTOR_CMD("M15GP", m15gpValue)
            _MATCH_MOTOR_CMD("M15R",  m15rValue)
            _MATCH_MOTOR_CMD("M15I",  m15iValue)
            _MATCH_MOTOR_CMD("M15M",  m15mValue)
            _MATCH_MOTOR_CMD("M15S",  m15sValue)
            _MATCH_MOTOR_CMD("M15A",  m15aValue)
            _MATCH_MOTOR_CMD("M15",   m15Value)
            // M17
            _MATCH_MOTOR_CMD("M17GP", m17gpValue)
            _MATCH_MOTOR_CMD("M17R",  m17rValue)
            _MATCH_MOTOR_CMD("M17I",  m17iValue)
            _MATCH_MOTOR_CMD("M17M",  m17mValue)
            _MATCH_MOTOR_CMD("M17S",  m17sValue)
            _MATCH_MOTOR_CMD("M17A",  m17aValue)
            _MATCH_MOTOR_CMD("M17",   m17Value)
            // M18
            _MATCH_MOTOR_CMD("M18GP", m18gpValue)
            _MATCH_MOTOR_CMD("M18R",  m18rValue)
            _MATCH_MOTOR_CMD("M18I",  m18iValue)
            _MATCH_MOTOR_CMD("M18M",  m18mValue)
            _MATCH_MOTOR_CMD("M18S",  m18sValue)
            _MATCH_MOTOR_CMD("M18A",  m18aValue)
            _MATCH_MOTOR_CMD("M18",   m18Value)
            #undef _MATCH_MOTOR_CMD
        }

        // DCM1/DCM2/DCW1/DCW2/DCS1/DCS2/DCF1..DCF4 — digit-suffix keys would
        // otherwise be truncated by the generic letters-only key extractor.
        // Plain DCM/DCD/DCF have no digits and are handled by the equals-block below.
        // Longer prefixes first so DCM1 doesn't match DCM, etc.
        {
            String tp = pair; tp.trim();
            #define _MATCH_DC_CMD(PREFIX, FIELD)                     \
                if (tp.startsWith(PREFIX)) {                         \
                    String tail = tp.substring(strlen(PREFIX));      \
                    tail.trim();                                     \
                    FIELD = (tail.length() == 0) ? 1 : tail.toInt(); \
                    continue;                                        \
                }
            _MATCH_DC_CMD("DCM1", dcm1Value)
            _MATCH_DC_CMD("DCM2", dcm2Value)
            _MATCH_DC_CMD("DCW1", dcw1Value)
            _MATCH_DC_CMD("DCW2", dcw2Value)
            _MATCH_DC_CMD("DCS1", dcs1Value)
            _MATCH_DC_CMD("DCS2", dcs2Value)
            _MATCH_DC_CMD("DCF1", dcf1Value)
            _MATCH_DC_CMD("DCF2", dcf2Value)
            _MATCH_DC_CMD("DCF3", dcf3Value)
            _MATCH_DC_CMD("DCF4", dcf4Value)
            #undef _MATCH_DC_CMD
        }

        {
            // UART println() leaves a trailing CR after the LF is consumed.
            String paraCommand = pair;
            paraCommand.trim();
            if (paraCommand == "PARA CHECK") { paraCheckValue = 1; continue; }
            if (paraCommand == "PARA STOP") { paraCheckValue = 0; continue; }
        }

        // Cascade commands that contain digits — handled before the generic letters-only key extractor.
        {
            String tp = pair; tp.trim();
            #define _MATCH_DIGIT_CMD(PREFIX, FIELD)                  \
                if (tp.startsWith(PREFIX)) {                         \
                    String tail = tp.substring(strlen(PREFIX));      \
                    tail.trim();                                     \
                    FIELD = (tail.length() == 0) ? 1 : tail.toInt(); \
                    continue;                                        \
                }
            _MATCH_DIGIT_CMD("S2MIX",  s2mixValue)
            _MATCH_DIGIT_CMD("B1MIX",  b1mixValue)
            // Legacy continuous pump start/stop keys. Execution disables starts;
            // STOP remains available as a safety stop. STOP before DIS so
            // "S1STOP" doesn't get partly-matched by "S1DIS"-style prefix scans
            // (they don't share a prefix today, but the ordering keeps it safe).
            _MATCH_DIGIT_CMD("S1STOP", s1stopValue)
            _MATCH_DIGIT_CMD("S1DIS",  s1disValue)
            _MATCH_DIGIT_CMD("S2STOP", s2stopValue)
            _MATCH_DIGIT_CMD("S2DIS",  s2disValue)
            _MATCH_DIGIT_CMD("B1STOP", b1stopValue)
            _MATCH_DIGIT_CMD("B1DIS",  b1disValue)
            _MATCH_DIGIT_CMD("B2STOP", b2stopValue)
            _MATCH_DIGIT_CMD("B2DIS",  b2disValue)
            // Profile-driven dispense triggers (master fills volume from active profile).
            // Longer prefixes first so DSPS1/DSPS2/DSPB1/DSPB2 win over a bare "DSP".
            _MATCH_DIGIT_CMD("DSPS1",  dsps1Value)
            _MATCH_DIGIT_CMD("DSPS2",  dsps2Value)
            _MATCH_DIGIT_CMD("DSPB1",  dspb1Value)
            _MATCH_DIGIT_CMD("DSPB2",  dspb2Value)
            _MATCH_DIGIT_CMD("DSPE",   dspeValue)
            #undef _MATCH_DIGIT_CMD
        }

        // PROFILE — accepts numeric ID (0..3) or 2-letter code (RP/LM/MG/WG).
        //   "PROFILE"      -> -1 (query mode: handler prints current profile)
        //   "PROFILE 2"    -> 2  (sets PROFILE_MG)
        //   "PROFILE MG"   -> 2  (string code resolved here)
        //   "PROFILE=WG"   -> 3
        //   "PROFILE XX"   -> -2 (invalid code)
        {
            String tp = pair; tp.trim();
            if (tp.equals("PROFILE")) {
                profileValue = -1;        // query
                continue;
            }
            if (tp.startsWith("PROFILE")) {
                String tail = tp.substring(7);          // drop "PROFILE"
                if (tail.startsWith("=")) tail = tail.substring(1);
                tail.trim();
                if (tail.length() == 0) { profileValue = -1; continue; }
                // Try numeric first
                bool numeric = true;
                for (int i = 0; i < tail.length(); ++i) {
                    if (!isDigit(tail.charAt(i)) && !(i == 0 && tail.charAt(i) == '-')) { numeric = false; break; }
                }
                if (numeric) {
                    long n = tail.toInt();
                    profileValue = (n >= 0 && n < 4) ? n : -2;
                } else {
                    tail.toUpperCase();
                    if      (tail.equals("RP")) profileValue = 0;
                    else if (tail.equals("LM")) profileValue = 1;
                    else if (tail.equals("MG")) profileValue = 2;
                    else if (tail.equals("WG")) profileValue = 3;
                    else                        profileValue = -2;
                }
                continue;
            }
        }

        // Process the pair - handle both formats (with and without equals sign)
        String key;
        int equalsIndex = pair.indexOf('=');

        if (equalsIndex != -1)
        {
            // Format with equals sign (X=10)
            key = pair.substring(0, equalsIndex);
        }
        else
        {
            // Format without equals sign (X10)
            // Extract the key (letters) from the beginning of the string
            int i = 0;
            while (i < pair.length() && !isDigit(pair.charAt(i)) && pair.charAt(i) != '-')
            {
                i++;
            }
            key = pair.substring(0, i);
        }

        key.trim();
        key.toUpperCase(); // Accept both lowercase and uppercase commands

        // General Commands
        if (key.equals("ID")) idValue = extractValue(pair);
        else if (key.equals("HELP")) helpValue = extractValue(pair);
        else if (key.equals("ACK")) ackValue = extractValue(pair);
        else if (key.equals("DEB")) debValue = extractValue(pair);
        else if (key.equals("MUTA")) magazineUartTestValue = 1;
        else if (key.equals("MUTB")) magazineUartTestValue = 2;
        else if (key.equals("MUTALL")) magazineUartTestValue = 3;

        // TMC Stepper Motor X
        else if (key.equals("X")) xValue = extractValue(pair);
        else if (key.equals("XR")) xrValue = extractValue(pair);
        else if (key.equals("XI")) xiValue = extractValue(pair);
        else if (key.equals("XM")) xmValue = extractValue(pair);
        else if (key.equals("XS")) xsValue = extractValue(pair);
        else if (key.equals("XA")) xaValue = extractValue(pair);
        else if (key.equals("XGP")) xgpValue = extractValue(pair);
        else if (key.equals("XML")) xmlValue = extractValue(pair);

        // TMC Stepper Motor Y
        else if (key.equals("Y")) yValue = extractValue(pair);
        else if (key.equals("YR")) yrValue = extractValue(pair);
        else if (key.equals("YI")) yiValue = extractValue(pair);
        else if (key.equals("YM")) ymValue = extractValue(pair);
        else if (key.equals("YS")) ysValue = extractValue(pair);
        else if (key.equals("YA")) yaValue = extractValue(pair);
        else if (key.equals("YGP")) ygpValue = extractValue(pair);
        else if (key.equals("YML")) ymlValue = extractValue(pair);
        else if (key.equals("SXCAS")) sxcasValue = extractValue(pair);
        else if (key.equals("WYCAS")) wycasValue = extractValue(pair);
        else if (key.equals("MLOAD"))   mloadValue   = extractValue(pair);
        else if (key.equals("MUNLOAD")) munloadValue = extractValue(pair);
        else if (key.equals("DRAIN"))   drainValue   = extractValue(pair);
        else if (key.equals("MIXM"))    mixmValue    = extractValue(pair);
        else if (key.equals("MIXD"))    mixdValue    = extractValue(pair);
        else if (key.equals("MIXR"))    mixrValue    = extractValue(pair);
        else if (key.equals("ETDIS"))   etdisValue   = extractValue(pair);
        else if (key.equals("ETSTOP"))  etstopValue  = extractValue(pair);

        // TMC Stepper Motor T
        else if (key.equals("T")) tValue = extractValue(pair);
        else if (key.equals("TR")) trValue = extractValue(pair);
        else if (key.equals("TI")) tiValue = extractValue(pair);
        else if (key.equals("TM")) tmValue = extractValue(pair);
        else if (key.equals("TS")) tsValue = extractValue(pair);
        else if (key.equals("TA")) taValue = extractValue(pair);
        else if (key.equals("TGP")) tgpValue = extractValue(pair);

        // TMC Stepper Motor Z
        else if (key.equals("Z"))   zValue   = extractValue(pair);
        else if (key.equals("ZR"))  zrValue  = extractValue(pair);
        else if (key.equals("ZI"))  ziValue  = extractValue(pair);
        else if (key.equals("ZM"))  zmValue  = extractValue(pair);
        else if (key.equals("ZS"))  zsValue  = extractValue(pair);
        else if (key.equals("ZA"))  zaValue  = extractValue(pair);
        else if (key.equals("ZGP")) zgpValue = extractValue(pair);
        else if (key.equals("ZML")) zmlValue = extractValue(pair);

        // (M14/M15/M17/M18 are handled at the top of the loop via prefix match,
        //  because the generic parser would chop their keys at the first digit.)

        // Gantry-X
        else if (key.equals("GX")) gxValue = extractValue(pair);
        else if (key.equals("GXR")) gxrValue = extractValue(pair);
        else if (key.equals("GXI")) gxiValue = extractValue(pair);
        else if (key.equals("GXM")) gxmValue = extractValue(pair);
        else if (key.equals("GXS")) gxsValue = extractValue(pair);
        else if (key.equals("GXA")) gxaValue = extractValue(pair);
        else if (key.equals("GXHO")) gxhoValue = extractValue(pair);
        else if (key.equals("GXGP")) gxgpValue = extractValue(pair);

        // Gantry-Y
        else if (key.equals("GY")) gyValue = extractValue(pair);
        else if (key.equals("GYR")) gyrValue = extractValue(pair);
        else if (key.equals("GYI")) gyiValue = extractValue(pair);
        else if (key.equals("GYM")) gymValue = extractValue(pair);
        else if (key.equals("GYS")) gysValue = extractValue(pair);
        else if (key.equals("GYA")) gyaValue = extractValue(pair);
        else if (key.equals("GYHO")) gyhoValue = extractValue(pair);
        else if (key.equals("GYGP")) gygpValue = extractValue(pair);

        // Gantry-Z
        else if (key.equals("GZ")) gzValue = extractValue(pair);
        else if (key.equals("GZR")) gzrValue = extractValue(pair);
        else if (key.equals("GZI")) gziValue = extractValue(pair);
        else if (key.equals("GZM")) gzmValue = extractValue(pair);
        else if (key.equals("GZS")) gzsValue = extractValue(pair);
        else if (key.equals("GZA")) gzaValue = extractValue(pair);
        else if (key.equals("GZHO")) gzhoValue = extractValue(pair);
        else if (key.equals("GZGP")) gzgpValue = extractValue(pair);
        else if (key.equals("GZCAL")) gzcalValue = extractValue(pair);
        else if (key.equals("LDM1")) ldm1Value = extractValue(pair);
        else if (key.equals("LDM")) ldm1Value = extractValue(pair);

        // Gantry-R
        else if (key.equals("GR")) grValue = extractValue(pair);
        else if (key.equals("GRR")) grrValue = extractValue(pair);
        else if (key.equals("GRI")) griValue = extractValue(pair);
        else if (key.equals("GRM")) grmValue = extractValue(pair);
        else if (key.equals("GRS")) grsValue = extractValue(pair);
        else if (key.equals("GRA")) graValue = extractValue(pair);
        else if (key.equals("GRHO")) grhoValue = extractValue(pair);
        else if (key.equals("GRLD")) grldValue = extractValue(pair);
        else if (key.equals("GRGP")) grgpValue = extractValue(pair);
        else if (key.equals("GHOME"))    ghomeValue    = extractValue(pair);
        else if (key.equals("NZHO"))     nzhoValue     = extractValue(pair);
        else if (key.equals("SXHO"))     sxhoValue     = extractValue(pair);
        else if (key.equals("SYHO"))     syhoValue     = extractValue(pair);
        else if (key.equals("WXHO"))     wxhoValue     = extractValue(pair);
        else if (key.equals("WYHO"))     wyhoValue     = extractValue(pair);
        else if (key.equals("BXHO"))     bxhoValue     = extractValue(pair);
        else if (key.equals("FEED"))     feedValue     = extractValue(pair);
        else if (key.equals("FEEDSTOP")) feedstopValue = pair.equals("FEEDSTOP") ? 1 : extractValue(pair);
        else if (key.equals("BEDTEST"))  bedtestValue  = pair.equals("BEDTEST")  ? 1 : extractValue(pair);
        else if (key.equals("BTRESET"))  btresetValue  = pair.equals("BTRESET")  ? 1 : extractValue(pair);
        else if (key.equals("GLOAD")) gloadValue = extractValue(pair);
        else if (key.equals("LOAD")) loadValue = extractValue(pair);
        else if (key.equals("RLD")) rldValue = pair.equals("RLD") ? 1 : extractValue(pair);
        else if (key.equals("RST")) rstValue = pair.equals("RST") ? 1 : extractValue(pair);

        // Stain X
        else if (key.equals("SX")) sxValue = extractValue(pair);
        else if (key.equals("SXR")) sxrValue = extractValue(pair);
        else if (key.equals("SXI")) sxiValue = extractValue(pair);
        else if (key.equals("SXS")) sxsValue = extractValue(pair);
        else if (key.equals("SXA")) sxaValue = extractValue(pair);
        else if (key.equals("SXGP")) sxgpValue = extractValue(pair);    

        // Stain Y
        else if (key.equals("SY")) syValue = extractValue(pair);
        else if (key.equals("SYR")) syrValue = extractValue(pair);
        else if (key.equals("SYI")) syiValue = extractValue(pair);
        else if (key.equals("SYS")) sysValue = extractValue(pair);
        else if (key.equals("SYA")) syaValue = extractValue(pair);
        else if (key.equals("SYGP")) sygpValue = extractValue(pair);

        // Wash X
        else if (key.equals("WX")) wxValue = extractValue(pair);
        else if (key.equals("WXR")) wxrValue = extractValue(pair);
        else if (key.equals("WXI")) wxiValue = extractValue(pair);
        else if (key.equals("WXS")) wxsValue = extractValue(pair);
        else if (key.equals("WXA")) wxaValue = extractValue(pair);
        else if (key.equals("WXGP")) wxgpValue = extractValue(pair);    

        // Wash Y
        else if (key.equals("WY")) wyValue = extractValue(pair);
        else if (key.equals("WYR")) wyrValue = extractValue(pair);
        else if (key.equals("WYI")) wyiValue = extractValue(pair);
        else if (key.equals("WYS")) wysValue = extractValue(pair);
        else if (key.equals("WYA")) wyaValue = extractValue(pair);
        else if (key.equals("WYGP")) wygpValue = extractValue(pair);

        // Buffer X
        else if (key.equals("BX")) bxValue = extractValue(pair);
        else if (key.equals("BXR")) bxrValue = extractValue(pair);
        else if (key.equals("BXI")) bxiValue = extractValue(pair);
        else if (key.equals("BXS")) bxsValue = extractValue(pair);
        else if (key.equals("BXA")) bxaValue = extractValue(pair);
        else if (key.equals("BXGP")) bxgpValue = extractValue(pair);

        // Buffer Y
        else if (key.equals("BY")) byValue = extractValue(pair);
        else if (key.equals("BYR")) byrValue = extractValue(pair);
        else if (key.equals("BYI")) byiValue = extractValue(pair);
        else if (key.equals("BYS")) bysValue = extractValue(pair);
        else if (key.equals("BYA")) byaValue = extractValue(pair);
        else if (key.equals("BYGP")) bygpValue = extractValue(pair);

        // Common
        else if (key.equals("GI")) giValue = extractValue(pair);
        else if (key.equals("GM")) gmValue = extractValue(pair);
        else if (key.equals("GS")) gsValue = extractValue(pair);
        else if (key.equals("GA")) gaValue = extractValue(pair);

        else if (key.equals("H")) hValue = extractValue(pair);
        else if (key.equals("HS")) hsValue = extractValue(pair);
        else if (key.equals("SMC")) smcValue = extractValue(pair);

        // Read IR Sensor Values
        else if (key.equals("RIR")) rirValue = extractValue(pair);
        else if (key.equals("RDHT")) rdhtValue = 1;
        else if (key.equals("PIR")) pirValue = extractValue(pair);

        // Read Limit Switch values
        else if (key.equals("RLS")) rlsValue = extractValue(pair);
        else if (key.equals("PLS")) plsValue = extractValue(pair);

        // Hall Sensor Modules
        else if (key.equals("GXH")) gxhValue = extractValue(pair);
        else if (key.equals("GXHI")) gxhiValue = extractValue(pair);
        else if (key.equals("MH1")) mhValue = 1;
        else if (key.equals("MH2")) mh2SampleCountValue = extractValue(pair);
        else if (key.equals("MH")) mhValue = extractValue(pair);
        else if (key.equals("GZH")) gzhValue = extractValue(pair);
        else if (key.equals("GZHI")) gzhiValue = extractValue(pair);

        else if (key.equals("M1L")) m1lValue = extractValue(pair);
        else if (key.equals("M1U")) m1uValue = extractValue(pair);
        else if (key.equals("M2L")) m2lValue = extractValue(pair);
        else if (key.equals("M2U")) m2uValue = extractValue(pair);

        // Read Accelerometer
        else if (key.equals("RAM")) ramValue = extractValue(pair);
        else if (key.equals("PAM")) pamValue = extractValue(pair);

        // DC Motor
        else if (key.equals("DCM")) dcmValue = extractValue(pair);
        else if (key.equals("DCM1")) dcm1Value = extractValue(pair);
        else if (key.equals("DCM2")) dcm2Value = extractValue(pair);
        else if (key.equals("DCD")) dcdValue = extractValue(pair);
        else if (key.equals("DCW1")) dcw1Value = extractValue(pair);
        else if (key.equals("DCW2")) dcw2Value = extractValue(pair);
        else if (key.equals("DCS1")) dcs1Value = extractValue(pair);
        else if (key.equals("DCS2")) dcs2Value = extractValue(pair);

        // DC Fan
        else if (key.equals("DCF1")) dcf1Value = extractValue(pair);
        else if (key.equals("DCF2")) dcf2Value = extractValue(pair);
        else if (key.equals("DCF3")) dcf3Value = extractValue(pair);
        else if (key.equals("DCF4")) dcf4Value = extractValue(pair);
        else if (key.equals("DCF")) dcfValue = extractValue(pair);

        else if (key.equals("M")) {
            if (pair.equals("M1L")) m1lValue = 1;
            else if (pair.equals("M1U")) m1uValue = 1;
            else if (pair.equals("M2L")) m2lValue = 1;
            else if (pair.equals("M2U")) m2uValue = 1;
        }

        else {
            Serial.print("Unknown_Key:");
            Serial.println(key);
        }
    }
    return true;
}

long Protocol2019Handler::extractValue(String pair)
{
    int equalsIndex = pair.indexOf('=');

    if (equalsIndex != -1)
    {
        // Format with equals sign (X=10)
        if (equalsIndex < pair.length() - 1)
        {
            String valueStr = pair.substring(equalsIndex + 1);
            valueStr.trim();
            return valueStr.toInt();
        }
    }
    else
    {
        // Format without equals sign (X10)
        // Find the first digit or minus sign
        int i = 0;
        while (i < pair.length() && !isDigit(pair.charAt(i)) && pair.charAt(i) != '-')
        {
            i++;
        }

        if (i < pair.length())
        {
            String valueStr = pair.substring(i);
            valueStr.trim();
            return valueStr.toInt();
        }
    }

    return 0;
}

void Protocol2019Handler::resetValues()
{
    // General Commands
    idValue     = ignoreValue;  
    helpValue   = ignoreValue;
    ackValue    = ignoreValue;
    debValue    = ignoreValue;
    magazineUartTestValue = ignoreValue;

    // TMC Stepper Motor X
    xValue   = ignoreValue;
    xrValue  = ignoreValue;
    xiValue  = ignoreValue;
    xmValue  = ignoreValue;
    xsValue  = ignoreValue;
    xaValue  = ignoreValue;
    xgpValue = ignoreValue;
    xmlValue = ignoreValue;

    // TMC Stepper Motor Y
    yValue   = ignoreValue;
    yrValue  = ignoreValue;
    yiValue  = ignoreValue;
    ymValue  = ignoreValue;
    ysValue  = ignoreValue;
    yaValue  = ignoreValue;
    ygpValue = ignoreValue;
    ymlValue = ignoreValue;
    sxcasValue = ignoreValue;
    s2mixValue = ignoreValue;
    b1mixValue = ignoreValue;
    wycasValue = ignoreValue;
    profileValue = ignoreValue;
    dspeValue   = ignoreValue;
    dsps1Value  = ignoreValue;
    dspb1Value  = ignoreValue;
    dsps2Value  = ignoreValue;
    dspb2Value  = ignoreValue;
    mloadValue   = ignoreValue;
    munloadValue = ignoreValue;
    drainValue   = ignoreValue;
    mixmValue    = ignoreValue;
    mixdValue    = ignoreValue;
    mixrValue    = ignoreValue;
    etdisValue   = etstopValue = ignoreValue;
    s1disValue   = s1stopValue = ignoreValue;
    s2disValue   = s2stopValue = ignoreValue;
    b1disValue   = b1stopValue = ignoreValue;
    b2disValue   = b2stopValue = ignoreValue;

    // Gantry-X
    gxValue   = ignoreValue;
    gxrValue  = ignoreValue;
    gxiValue  = ignoreValue;
    gxmValue  = ignoreValue;
    gxsValue  = ignoreValue;
    gxaValue  = ignoreValue;
    gxhoValue = ignoreValue;
    gxgpValue = ignoreValue;
    
    // Gantry-Y
    gyValue   = ignoreValue;
    gyrValue  = ignoreValue;
    gyiValue  = ignoreValue;
    gymValue  = ignoreValue;
    gyhoValue = ignoreValue;
    gysValue  = ignoreValue;
    gyaValue  = ignoreValue;
    gygpValue = ignoreValue;
   
    // Gantry-Z
    gzValue   = ignoreValue;
    gzrValue  = ignoreValue;
    gziValue  = ignoreValue;
    gzmValue  = ignoreValue;
    gzsValue  = ignoreValue;
    gzaValue  = ignoreValue;
    gzhoValue = ignoreValue;
    gzgpValue = ignoreValue;
    gzcalValue= ignoreValue;
    ldm1Value = ignoreValue;

    // Gantry-R
    grValue   = ignoreValue;
    grrValue  = ignoreValue;
    griValue  = ignoreValue;
    grmValue  = ignoreValue;
    grsValue  = ignoreValue;
    graValue  = ignoreValue;
    grhoValue = ignoreValue;
    grldValue = ignoreValue;
    grgpValue = ignoreValue;
    ghomeValue    = ignoreValue;
    nzhoValue     = ignoreValue;
    sxhoValue     = ignoreValue;
    syhoValue     = ignoreValue;
    wxhoValue     = ignoreValue;
    wyhoValue     = ignoreValue;
    bxhoValue     = ignoreValue;
    feedValue     = ignoreValue;
    feedstopValue = ignoreValue;
    bedtestValue  = ignoreValue;
    paraCheckValue = ignoreValue;
    btresetValue  = ignoreValue;
    gloadValue    = ignoreValue;
    loadValue = ignoreValue;
    rldValue = ignoreValue;
    rstValue = ignoreValue;

    // Stain X
    sxValue   = ignoreValue;
    sxrValue  = ignoreValue;
    sxiValue  = ignoreValue;
    sxsValue  = ignoreValue;
    sxaValue  = ignoreValue;
    sxgpValue = ignoreValue;

    // Stain Y
    syValue   = ignoreValue;
    syrValue  = ignoreValue;
    syiValue  = ignoreValue;
    sysValue  = ignoreValue;
    syaValue  = ignoreValue;
    sygpValue = ignoreValue;

    // Wash X
    wxValue   = ignoreValue;
    wxrValue  = ignoreValue;
    wxiValue  = ignoreValue;
    wxsValue  = ignoreValue;
    wxaValue  = ignoreValue;
    wxgpValue = ignoreValue;

    // Wash Y
    wyValue   = ignoreValue;
    wyrValue  = ignoreValue;
    wyiValue  = ignoreValue;
    wysValue  = ignoreValue;
    wyaValue  = ignoreValue;
    wygpValue = ignoreValue;

    // Buffer X
    bxValue   = ignoreValue;
    bxrValue  = ignoreValue;
    bxiValue  = ignoreValue;
    bxsValue  = ignoreValue;
    bxaValue  = ignoreValue;
    bxgpValue = ignoreValue;

    // Buffer Y
    byValue   = ignoreValue;
    byrValue  = ignoreValue;
    byiValue  = ignoreValue;
    bysValue  = ignoreValue;
    byaValue  = ignoreValue;
    bygpValue = ignoreValue;

    // Z Motor
    zValue   = ignoreValue;
    zrValue  = ignoreValue;
    ziValue  = ignoreValue;
    zmValue  = ignoreValue;
    zsValue  = ignoreValue;
    zaValue  = ignoreValue;
    zgpValue = ignoreValue;
    zmlValue = ignoreValue;

    // T Motor
    tValue   = ignoreValue;
    trValue  = ignoreValue;
    tiValue  = ignoreValue;
    tmValue  = ignoreValue;
    tsValue  = ignoreValue;
    taValue  = ignoreValue;
    tgpValue = ignoreValue;

    // M14 / M15 / M17 / M18 Motors
    m14Value = m14rValue = m14iValue = m14mValue = m14sValue = m14aValue = m14gpValue = m14mlValue = ignoreValue;
    m15Value = m15rValue = m15iValue = m15mValue = m15sValue = m15aValue = m15gpValue = m15mlValue = ignoreValue;
    m17Value = m17rValue = m17iValue = m17mValue = m17sValue = m17aValue = m17gpValue = ignoreValue;
    m18Value = m18rValue = m18iValue = m18mValue = m18sValue = m18aValue = m18gpValue = ignoreValue;

    // Common
    giValue  = ignoreValue;
    gmValue  = ignoreValue;
    gsValue  = ignoreValue;
    gaValue  = ignoreValue;

    hValue   = ignoreValue;

    hsValue  = ignoreValue;

    smcValue = ignoreValue;

    // Read IR Sensor values
    rirValue = ignoreValue;
    rdhtValue = ignoreValue;
    pirValue = ignoreValue;

    // Read Limit Switch values
    rlsValue = ignoreValue;
    plsValue = ignoreValue;

    // Hall sensor Modules
    gxhValue  = ignoreValue;
    gxhiValue = ignoreValue;
    mhValue   = ignoreValue;
    mh2SampleCountValue = ignoreValue;
    mhp2SampleCountValue = ignoreValue;
    gzhValue  = ignoreValue;
    gzhiValue = ignoreValue;

    m1lValue = ignoreValue;
    m1uValue = ignoreValue;
    m2lValue = ignoreValue;
    m2uValue = ignoreValue;

    cal1Value = ignoreValue;
    cal2Value = ignoreValue;

    // Read Accelerometer
    ramValue = ignoreValue;
    pamValue = ignoreValue;

    // DC Motor
    dcmValue = ignoreValue;
    dcm1Value = ignoreValue;
    dcm2Value = ignoreValue;
    dcdValue  = ignoreValue;
    dcw1Value = ignoreValue;
    dcw2Value = ignoreValue;
    dcs1Value = ignoreValue;
    dcs2Value = ignoreValue;

    // DC Fan
    dcf1Value = ignoreValue;
    dcf2Value = ignoreValue;
    dcf3Value = ignoreValue;
    dcf4Value = ignoreValue;
    dcfValue  = ignoreValue;
}

void Protocol2019Handler::process2019Packet()
{
    execution2019Handler.displayPacketValues();
    execution2019Handler.performPacketActions();

    // Reset after processing
    resetValues();
}

bool Protocol2019Handler::hasKey(ByteArrayHandler &packet, const String &key)
{
    String packetStr = "";
    for (int i = 0; i < packet.getSize(); i++)
    {
        packetStr += (char)packet.getByte(i);
    }

    // Check if the key exists in the packet
    int keyIndex = packetStr.indexOf(key + "=");
    return (keyIndex != -1);
}
