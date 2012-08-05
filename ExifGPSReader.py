import unittest
import code
import collections

class ExifFormatException(Exception):
    pass

class ExifGPSReader(object):
    """
    Provides a function that parses the passed-in stream until it has
    either found all of a subset of GPS-related EXIF tags or until it
    reaches the end of the APP1 layer. Expects the stream to be reading
    a well-formed JPG. You have to open and close the stream yourself.
    
    The stream passed in can be bufferless as the function only reads
    in a forward direction and never returns to a previously visited
    offset.
    
    The GPS EXIF tags this object cares about are:
    - Latitude Reference
    - Latitude
    - Longitude Reference
    - Longitude
    - Altitude Reference
    - Altitude
    - Direction Reference
    - Direction
    - Timestamp
    """
    
    SOI = '\xff\xd8'
    APP0 = '\xff\xe0'
    APP1 = '\xff\xe1'
    GPS_TAG = '\x88\x25'
    GPS_TAGS = {1: 'latitude_ref', 2: 'latitude', 3: 'longitude_ref',
                4: 'longitude', 5: 'altitude_ref', 6: 'altitude',
                7: 'timestamp', 16: 'direction_ref', 17: 'direction'}

    # Useful for identifying EXIF tags later on.
    ASCII_TAG = 2
    RATIONAL_TAG = 5

    # These are the only datatypes used by the EXIF tags we are
    # interested in. Refer to the EXIF spec if you want to expand this.
    DATATYPE_LENGTHS = {0: 0, # no such type
                        1: 1, # Byte
                        2: 1, # ASCII
                        5: 8} # Rational

    def __init__(self, stream):
        self._stream = stream

    def read_gps(self):
        """
        Reads the stream passed in at the object's creation until it
        either has found all the values for a subset of GPS EXIF tags,
        or until it finds the end of the APP1 layer. Returns whatever
        values it manages to find as a dictionary of strings to values.
        
        Currently uses floating point division for latitude, longitude,
        altitude, direction, and timestamp. These values are stored in
        the JPEG as "rationals," or pairs of longs with one long
        representing a denominator and the other representing a
        numerator. I don't care about accuracy *that* much, so I just
        convert these to floats and divide. If you want more accuracy,
        it would be a simple matter to modify this code to use Decimal
        objects instead, or tuples, or whatever else floats your boat.
        
        Throws ExifFormatExceptions if the image is missing the APP1
        tag, the start-of-image tag, or if the endianness marker is
        malformed.
        """
        gps = {}
        ifd_offsets = {}

        # I get tired of typing self._stream all the time.
        stream = self._stream

        # Check for the start-of-image tag.
        if (stream.read(2) != self.SOI):
            raise ExifFormatException("Missing start-of-image tag!")

        segment = stream.read(2)
        
        # Skip the APP0 layer if this JPG has one.
        if (segment == self.APP0):
            stream.seek(stream.tell()
                + (ord(stream.read(1)) * 256
                + ord(stream.read(1))))
            
            # Read the beginning tag of the next layer
            segment = stream.read(2)

        # Make sure we have an APP1 layer.
        if (segment != self.APP1):
            raise ExifFormatException("APP1 segment is missing!")

        # Get the offset at which APP1 ends so we can stop reading
        # the stream if we reach it.
        end_of_app1 = (stream.tell()
            + (ord(stream.read(1)) * 256
            + ord(stream.read(1))))

        # Skip the "Exif" opening string. Not important to us.
        stream.read(6)
        
        # We are at the start of the EXIF data!
        # Remember our offset since all tag offsets in this layer
        # will be relative to this offset.
        exif_start = stream.tell()
        
        # Figure out the endianness of this file and set our
        # evaluation function accordingly. Raise an exception if the
        # endianness cannot be determined. That is a deal-breaker.
        endian_string = stream.read(4)
             
        if endian_string == 'II*\x00':
            self._val = self._intel_val
        elif endian_string == 'MM\x00*':
            self._val = self._motorola_val
        else:
            raise ExifFormatException("Could not determine endianness.")

        # Find the beginning of the GPS tag.
        found_gps = False
        
        while (stream.tell() < end_of_app1 and not found_gps):
            segment = stream.read(2)
            found_gps = (segment == self.GPS_TAG)
        
        # Did we find any GPS info?
        if found_gps:
                
            # Skip the datatype and number of fields for the IFD field
            # count. We already know from the EXIF spec what these are.
            stream.read(6)
            stream.seek(exif_start + self._val(stream.read(4))) 
            
            # Get the number of IFD tags contained in this IFD.
            num_fields = self._val(stream.read(2))

            # Extract the actual IFD tags. 
            for i in range(num_fields):
                tag_id = self._val(stream.read(2))
                
                if tag_id in self.GPS_TAGS:
                    
                    # Get all the tag metadata.
                    tag = self.GPS_TAGS[tag_id]
                    datatype = self._val(stream.read(2))
                    datatype_length = self.DATATYPE_LENGTHS[datatype]
                    num_values = self._val(stream.read(4)) 
                    value_length = datatype_length * num_values

                    # For values over 4 bytes, the data gets stored
                    # later in the EXIF file. Values 4 bytes or under
                    # are stored here as zero-padded data.
                    if value_length > 4:
                        # Get 4 byte offset at which the tag's actual
                        # value is stored. Associate the absolute offset
                        # with the data we will need to get the value 
                        # once we actually reach that point in the
                        # stream. This is to accomodate the forward-only
                        # behavior I wanted in order to accomodate a
                        # bufferless stream.
                        ifd_offsets[exif_start
                            + self._val(stream.read(4))] = (tag,
                                                           num_values,
                                                           datatype)
                    else:
                        # We're assuming anything less than 4 bytes long
                        # will only be a single value. This is a safe
                        # assumption given the subset of GPS tags we are
                        # interested in. That's why we just throw the
                        # output from the stream into a list by itself
                        # and pass it to our _get_value method.
                        # More complicated handling may be required if
                        # you expand this reader to handle other tags.
                        gps[tag] = self._get_value(
                            [stream.read(4)[0 : value_length]],
                            num_values,
                            datatype)
                else:
                    # We don't recognize this tag? Skip it.
                    stream.read(10)
            
            # Alright, do we have any tag values that were over 4 bytes?
            # If so, let's keep reading the stream and retrieve them.
            if len(ifd_offsets) > 0:
                
                # First we sort the offsets to accomodate the forward-
                # only behavior we want.
                sorted_offsets = sorted(ifd_offsets.iterkeys())

                for offset in sorted_offsets:
                    stream.seek(offset)
                    
                    # Get the tag metadata we saved earlier.
                    tag, num_values, datatype = ifd_offsets[offset]

                    # We're not going to worry about the value length
                    # at this point. By definition, we are only dealing
                    # with values that are over 4 bytes long. Thus we
                    # don't have to worry about zero-padding.
                    # Grab each of the values associated with this tag.
                    gps[tag] = self._get_value(
                        [stream.read(datatype_length)
                            for i in range(num_values)],
                        num_values, datatype)
                        
        # end if found_gps
        
        return gps

    def _get_value(self, data, num_values, datatype):
        """
        Converts an array of hex strings to an array of values,
        or a single value if the array contains a single element.
        """
        # If this is an ASCII value, then the number of values in this
        # field corresponds to the number of characters in the string.
        # The data array itself is a single-element array. Thus we have
        # to handle ASCII strings quite differently from the other data
        # types.
        if datatype == self.ASCII_TAG:
            # Pull the string from the single-element array and strip
            # the null character at the end.
            data = data[0][0:-1]
            
        else:
            # Pick the appropriate evaluation function.
            if datatype == self.RATIONAL_TAG:
                val = self._rational_val
            else:
                val = self._val

            # Get the value of each hex string in the data array.
            for i in range(num_values):
                data[i] = val(data[i])

            # Don't return an array if there is only one value in the
            # array. Return the value instead.
            if num_values == 1:
                data = data[0]

        return data

    def _rational_val(self, data):
        """
        Takes an 8 element array representing an EXIF rational value,
        grabs the numerator and denominator, divides them as floats,
        and returns the result.
        """
        return (float(self._val(data[0:4])) / float(self._val(data[4:8])))
        
    def _motorola_val(self, string):
        """
        Takes a big-endian hex string and returns an integer value.
        """
        x = 0
        for c in string:
            x = (x << 8) | ord(c)
        return x

    def _intel_val(self, string):
        """
        Takes a little-endian hex string and returns an integer value.
        """
        x = 0
        y = 0
        for c in string:
            x = x | (ord(c) << y)
            y = y + 8
        return x

class TestExifReader(unittest.TestCase):
    """
    Just a simple test for the ExifReader. Requires the test.jpg file
    that should have been included with this file.
    """
    def setUp(self):
        self._reader = ExifGPSReader(open("test.jpg", "rb"))

    def test_read_gps(self):
        gps = self._reader.read_gps()
        self.assertEqual(gps, {'timestamp': [13.0, 0.0, 20.0], # 1:00:20 PM
                               'altitude': 10.465,
                               'altitude_ref': 0,
                               'longitude': [122.0, 5.0, 2.0796],
                               'longitude_ref': 'W',
                               'latitude': [37.0, 25.0, 19.6716],
                               'latitude_ref': 'N',
                               'direction': 60.588,
                               'direction_ref': 'T'})

if __name__ == '__main__':
    unittest.main()
