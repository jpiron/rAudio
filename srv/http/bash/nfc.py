#!/bin/python

# nfc.service > this: PN532 reader - NTAG21x tags
#   - reader : PN532 HSU (UART) on GPIO 14/15 - DIP switches 0 0
#   - tag    : NDEF message, 1 MIME record application/x-raudio
#       { "v": 1, "type": "folder", "path": "NAS/music/Artist/Album" }
#       { "v": 1, "type": "playlist", "name": "Playlist" }

import json
import os
import select
import termios
import time
import tty

MIME   = b'application/x-raudio'
KEY    = { 'folder': 'path', 'playlist': 'name' } # value key per type
STATUS = { # InDataExchange / InCommunicateThru error status
      0x01: 'Tag timeout'
    , 0x02: 'CRC error'
    , 0x14: 'Tag authentication failed'
    , 0x27: 'Tag command not supported'
    , 0x29: 'Tag removed'
}

class NfcError( Exception ):
    pass
# transport ------------------------------------------------------------
# interface for I2C / SPI later: send( frame ), read( n, deadline ) > bytes | TimeoutError, close()
def serialPort(): # tty on GPIO 14/15 - Arch has no /dev/serial0 > device tree alias
    try:
        with open( '/proc/device-tree/aliases/serial0', 'rb' ) as f:
            node = '/sys/firmware/devicetree/base'+ f.read().rstrip( b'\0' ).decode()
    except OSError:
        return '/dev/ttyAMA0'

    for name in sorted( os.listdir( '/sys/class/tty' ) ):
        of_node = '/sys/class/tty/'+ name +'/device/of_node'
        if os.path.exists( of_node ) and os.path.realpath( of_node ) == os.path.realpath( node ):
            return '/dev/'+ name

    return '/dev/ttyAMA0'

class Uart:
    WAKEUP = b'\x55\x55' + bytes( 14 )

    def __init__( self, port ):
        self.asleep = True
        self.fd     = os.open( port, os.O_RDWR | os.O_NOCTTY )
        tty.setraw( self.fd )
        attr        = termios.tcgetattr( self.fd )
        attr[ 2 ]   = ( attr[ 2 ] | termios.CLOCAL | termios.CREAD ) & ~termios.CRTSCTS
        attr[ 4 ]   = attr[ 5 ] = termios.B115200
        termios.tcsetattr( self.fd, termios.TCSANOW, attr )
        termios.tcflush( self.fd, termios.TCIOFLUSH )

    def send( self, frame ):
        termios.tcflush( self.fd, termios.TCIFLUSH )
        if self.asleep: frame = self.WAKEUP + frame # must be in the same write - no reply if delayed
        os.write( self.fd, frame )

    def read( self, n, deadline ):
        data = b''
        while len( data ) < n:
            timeout = deadline - time.monotonic()
            if timeout <= 0 or not select.select( [ self.fd ], [], [], timeout )[ 0 ]:
                raise TimeoutError( f'{ len( data ) }/{ n } bytes: { data.hex( " " ) }' )

            data += os.read( self.fd, n - len( data ) )
        return data

    def close( self ):
        os.close( self.fd )
# pn532 ----------------------------------------------------------------
def frame( data ): # 00 00 FF LEN LCS [ D4 CMD PARAMS ] DCS 00
    n = len( data )
    return bytes( [ 0x00, 0x00, 0xff, n, -n & 0xff ] ) + data + bytes( [ -sum( data ) & 0xff, 0x00 ] )

class Pn532:
    def __init__( self, io ):
        self.io = io

    def readFrame( self, timeout ): # b'' = ACK
        deadline = time.monotonic() + timeout
        last     = b''
        while last != b'\x00\xff': # preamble may have extra 00
            last = last[ -1: ] + self.io.read( 1, deadline )
        length, lcs = self.io.read( 2, deadline )
        if ( length, lcs ) == ( 0x00, 0xff ):
            self.io.read( 1, deadline ) # postamble
            return b''

        if ( length + lcs ) & 0xff: raise NfcError( f'PN532 frame: bad LCS {length:#04x} {lcs:#04x}' )

        data = self.io.read( length + 2, deadline )
        data, dcs = data[ :-2 ], data[ -2 ]
        if ( sum( data ) + dcs ) & 0xff: raise NfcError( 'PN532 frame: bad DCS '+ data.hex( ' ' ) )

        if data == b'\x7f': raise NfcError( 'PN532 frame: syntax error' )

        return data

    def call( self, cmd, params=b'', timeout=1 ):
        self.io.send( frame( bytes( [ 0xd4, cmd ] ) + params ) )
        try:
            if self.readFrame( 0.1 ) != b'': raise NfcError( f'PN532 {cmd:#04x}: no ACK' )

            data = self.readFrame( timeout )
        except TimeoutError as e:
            self.io.asleep = True # power-down or reset: wake up on next command
            raise NfcError( f'PN532 {cmd:#04x}: no reply ({ e })' ) from None

        if data[ :2 ] != bytes( [ 0xd5, cmd + 1 ] ): raise NfcError( f'PN532 {cmd:#04x}: bad reply '+ data.hex( ' ' ) )

        self.io.asleep = False
        return data[ 2: ]

    def begin( self ): # > 'PN532 v1.6'
        self.call( 0x14, b'\x01\x14\x01' )               # SAMConfiguration: normal mode - leave LowVbat
        self.call( 0x32, b'\x05\xff\x01\x02' )           # RFConfiguration: MxRtyPassiveActivation 2 - InListPassiveTarget returns
        ic, ver, rev, _ = self.call( 0x02 )              # GetFirmwareVersion
        return f'PN5{ic:02x} v{ ver }.{ rev }'

    def target( self ): # > None | ( tg, uid, sak ) - 1 ISO14443A target 106 kbps
        data = self.call( 0x4a, b'\x01\x00' )            # InListPassiveTarget
        if not data[ 0 ]: return None

        tg, sak, uidlen = data[ 1 ], data[ 4 ], data[ 5 ]
        return tg, data[ 6:6 + uidlen ], sak

    def exchange( self, tg, data ):
        reply  = self.call( 0x40, bytes( [ tg ] ) + data ) # InDataExchange - CRC added by PN532
        status = reply[ 0 ] & 0x3f
        if status: raise NfcError( STATUS.get( status, f'Tag error {status:#04x}' ) )

        return reply[ 1: ]

    def release( self, tg ):
        self.call( 0x52, bytes( [ tg ] ) )                # InRelease
# ntag -----------------------------------------------------------------
class Ntag: # NFC Forum Type 2 - data area from page 4, size from CC (page 3)
    def __init__( self, pn, tg, sak ):
        if sak != 0x00: raise NfcError( f'Unsupported tag (SAK {sak:#04x}): NTAG21x only' )

        self.pn   = pn
        self.tg   = tg
        cc        = self.read( 3 )[ :4 ]
        if cc[ 0 ] != 0xe1: raise NfcError( 'Tag not NDEF formatted' )

        self.size     = cc[ 2 ] * 8                   # NTAG213: 144, NTAG215: 496, NTAG216: 872
        self.last     = 3 + self.size // 4            # last data area page
        self.writable = cc[ 3 ] & 0x0f == 0

    def read( self, page ): # > 16 bytes - 4 pages
        return self.pn.exchange( self.tg, bytes( [ 0x30, page ] ) )

    def write( self, page, data ): # 4 bytes - data area only: never UID, lock, CC or config pages
        if not 4 <= page <= self.last or len( data ) != 4: raise NfcError( f'Write denied: page {page:#04x}' )

        self.pn.exchange( self.tg, bytes( [ 0xa2, page ] ) + data )

    def readNdef( self ): # > NDEF message | b'' - TLV: 00 null, 03 NDEF, FE terminator, others skipped
        area = b''
        def get( end ): # area[ :end ] - read only needed pages
            nonlocal area
            if end > self.size: raise NfcError( 'Invalid TLV: beyond data area' )

            while len( area ) < end:
                area += self.read( 4 + len( area ) // 4 )
            return area

        i = 0
        while i < self.size:
            t = get( i + 1 )[ i ]
            if t == 0x00:
                i += 1
                continue

            if t == 0xfe: break

            length = get( i + 2 )[ i + 1 ]
            start  = i + 2
            if length == 0xff:
                length = int.from_bytes( get( i + 4 )[ i + 2:i + 4 ], 'big' )
                start  = i + 4
            if t == 0x03: return get( start + length )[ start:start + length ]

            i = start + length
        return b''

    def writeNdef( self, message ):
        n    = len( message )
        tlv  = b'\x03' + ( n < 0xff and bytes( [ n ] ) or b'\xff' + n.to_bytes( 2, 'big' ) ) + message + b'\xfe'
        if not self.writable: raise NfcError( 'Tag is read-only' )

        if len( tlv ) > self.size: raise NfcError( f'Data too large for tag: { len( tlv ) }/{ self.size } bytes' )

        tlv  += bytes( -len( tlv ) % 4 )
        pages = [ tlv[ i:i + 4 ] for i in range( 0, len( tlv ), 4 ) ]
        if len( pages ) > 1:
            self.write( 4, b'\x03\x00\xfe\x00' )          # empty NDEF first: tag removed while writing > empty, not corrupted
            for i, data in enumerate( pages[ 1: ], 5 ):
                self.write( i, data )
        self.write( 4, pages[ 0 ] )
        if self.readNdef() != message: raise NfcError( 'Verify failed' )
# ndef -----------------------------------------------------------------
def ndefEncode( type, value ): # > NDEF message - 1 MIME record
    payload = json.dumps( { 'v': 1, 'type': type, KEY[ type ]: value }, separators=( ',', ':' ), ensure_ascii=False ).encode()
    n       = len( payload )
    sr      = n < 0x100
    header  = 0xc0 | ( sr and 0x10 ) | 0x02           # MB ME [SR] TNF=MIME
    return bytes( [ header, len( MIME ) ] ) + ( sr and bytes( [ n ] ) or n.to_bytes( 4, 'big' ) ) + MIME + payload

def ndefDecode( message ): # > { type, path | name }
    if not message: raise NfcError( 'Tag is empty' )

    i = 0
    try:
        while True:
            header  = message[ i ]
            tlen    = message[ i + 1 ]
            i      += 2
            if header & 0x10: # SR
                plen = message[ i ]
                i   += 1
            else:
                plen = int.from_bytes( message[ i:i + 4 ], 'big' )
                i   += 4
            ilen    = header & 0x08 and message[ i ] or 0 # IL
            i      += header & 0x08 and 1
            rtype   = message[ i:i + tlen ]
            i      += tlen + ilen
            payload = message[ i:i + plen ]
            i      += plen
            if i > len( message ): raise IndexError

            if header & 0x27 == 0x02 and rtype == MIME: return ndefTarget( payload ) # TNF=MIME, not chunked

            if header & 0x40 or i >= len( message ): break # ME
    except IndexError:
        raise NfcError( 'Invalid NDEF message' ) from None

    raise NfcError( 'No rAudio data on tag' )

def ndefTarget( payload ):
    try:
        data = json.loads( payload )
    except ValueError:
        raise NfcError( 'Invalid rAudio data' ) from None

    if not isinstance( data, dict ) or data.get( 'v' ) != 1: raise NfcError( f'Unsupported rAudio data version: { isinstance( data, dict ) and data.get( "v" ) }' )

    type  = data.get( 'type' )
    key   = KEY.get( type )
    value = key and data.get( key )
    if not value or not isinstance( value, str ): raise NfcError( f'Invalid rAudio data: { payload.decode( errors="replace" ) }' )

    return { 'type': type, key: value }
