const NFCA_MIN_LISTEN_FDT: u32 = 1172;

/// Frame delay time (FDT) adjustment
const ST25R3911B_FDT_ADJUST: u32 = 64;

/** According to Digital specification 6.10 the dead rx time should be
 * 1172 - 128 1/fc  and refer to STM25R3911B, MRT timer start after EOF so the
 * 128 1/fc + 20 1/fc have to be also subtracted.
 */
const NFCA_POLL_FTD_ADJUSMENT: u32 = 276;
const NFCA_FWT_A_ADJUSMENT: u32 = 512;

pub const MASK_RECEIVE_TIMER: u32 =
    NFCA_MIN_LISTEN_FDT - (ST25R3911B_FDT_ADJUST + NFCA_POLL_FTD_ADJUSMENT);

pub const NO_RESPONSE_TIMER: u32 = NFCA_MIN_LISTEN_FDT + ST25R3911B_FDT_ADJUST + NFCA_FWT_A_ADJUSMENT;
