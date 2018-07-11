/*	Open Bionics - CircleBuff
*	Author - Olly McBride
*	Date - February 2017
*
*	This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
*	To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/.
*
*	Website - http://www.openbionics.com/
*	GitHub - https://github.com/Open-Bionics
*	Email - ollymcbride@openbionics.com
*
*	CircleBuff.h
*
*/

#ifndef CIRCLE_BUFFER_H_
#define CIRCLE_BUFFER_H_

#include <stdint.h>
#include <stdlib.h>

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

// MUST BE A POWER OF 2
//#define C_BUFF_MAX_SIZE		64				// default buffer size  (2^6)
//#define C_BUFF_MAX_SIZE		128				// default buffer size  (2^7)
//#define C_BUFF_MAX_SIZE		255				// default buffer size  (2^8)
//#define C_BUFF_MAX_SIZE		512				// default buffer size  (2^9)
#define C_BUFF_MAX_SIZE			1024			// default buffer size  (2^10)
//#define C_BUFF_MAX_SIZE		2048			// default buffer size  (2^11)
//#define C_BUFF_MAX_SIZE		4096			// default buffer size  (2^12)
//#define C_BUFF_MAX_SIZE		8192			// default buffer size  (2^13)


/**
* # Circle buffer class
*
* The circle buffer unit is a buffer in which data can be stored. The buffer is instantiated to a 
* set type and initialised to a set size, where data can then be stored. Each new data entry is 
* stored in the next buffer 'slot', and when the buffer is full, the buffer wraps around and overwrites 
* the first data entry. Thus the buffer will not overflow.
*
* The circle buffer also calculates a running total, running mean, minimum and maximum value of 
* all data entries.
*
*/

template <class buff_type> 
class CIRCLE_BUFFER
{
	public:
		CIRCLE_BUFFER()
		{
			_buffSize = 0;			/**< size of the buffer (must be a power of 2) */
			_buffSizeMask = 0;		/**< mask used to calculate the wrap around point */
			_buff = nullptr;		/**< data buffer */

			_readIndex = 0;			/**< index to read data from */
			_writeIndex = 0;		/**< index to write new data to */

			_total = 0;				/**< running total */
			_mean = 0;				/**< running mean */
			_min = (sizeof(buff_type) * UINT8_MAX);;			/**< all-time minimum value */
			_max = 0;				/**< all-time maximum value */
		}

		~CIRCLE_BUFFER()
		{
			destroy();
		}

		/**  Create an instance of a circular buffer of size 'size' (size must be a power or 2)
		*/
		bool begin(uint16_t size)				
		{
			// if buffer size is not a power of 2, increase size until it is
			while (!isPowerOfTwo(size))
			{
				size++;
			}

			// limit array size to maximum allocated size
			if (size > C_BUFF_MAX_SIZE)
			{
				size = C_BUFF_MAX_SIZE;
			}

			// allocate memory for buffer
			_buff = (buff_type*) malloc(size * sizeof(buff_type));

			// set circle buffer size
			_buffSize = size;
			_buffSizeMask = (_buffSize - 1);

			// reset read/write index
			_readIndex = 0;
			_writeIndex = 0;

			// clear maths variables
			_total = 0;
			_mean = 0;

			if (_buff == NULL)
			{
				return false;
			}
			else
			{
				// initialise all values to 0, so that when it is subtracted from the total on the first write nothing happens
				fill(0);

				// reset the max & min values after the fill so that min != 0
				resetMaxMin();

				return true;
			}
		}

		/**  Fill buffer with an initial value
		*/
		void fill(buff_type value)				
		{
			_total = (value * _buffSize);
			_mean = value;
			_min = value;
			_max = value;

			for (uint16_t i = 0; i < _buffSize; i++)
			{
				_buff[i] = value;
			}
		}

		/**  Read the first (oldest) value from the buffer and increment read index
		*/
		buff_type read(void)					
		{
			if (_buff == NULL)
				return (buff_type)0;

			return _buff[_readIndex++ & _buffSizeMask];
		}

		/**  Read the first (oldest) value from the buffer and don't increment read index
		*/
		buff_type readGlimpse(void)				
		{
			if (_buff == NULL)
				return (buff_type)0;

			return _buff[_readIndex & _buffSizeMask];
		}

		/**  Read a specific element within the buffer
		*/
		buff_type readElement(uint16_t index)	
		{
			if (_buff == NULL)
				return (buff_type)0;

			return _buff[index];
		}

		/**  Read the mean of the buffer
		*/
		buff_type readMean(void)				
		{
			return _mean;
		}

		/**  Read the total of the buffer
		*/
		long double readTotal(void)
		{
			return _total;
		}

		/**  Read the minimum value of the buffer
		*/
		buff_type readMin(void)					
		{
			return _min;
		}

		/**  Read the maximum value of the buffer
		*/
		buff_type readMax(void)					
		{
			return _max;
		}

		/**  Write to the buffer, recalculate the total and the mean (and std dev if enabled)
		*/
		bool write(buff_type value)				
		{
			if (_buff == NULL)
				return false;

			_min = min(_min, value);
			_max = max(_max, value);

			// remove first value from total, then add new value and calculate mean
			_total -= (long double)_buff[_writeIndex & _buffSizeMask];		// remove last value from total
			_total += (long double) value;									// add new value to total

			// calculate new mean
			if (_total == 0)
			{
				_mean = 0;
			}
			else
			{
				_mean = (long double)_total / (long double)_buffSize;
			}

			_buff[_writeIndex++ & _buffSizeMask] = value;				// add new value to buffer	

			// return true to indicate successful write
			return true;
		}


		/**  Get the actual buffer size (power of 2)
		*/
		uint16_t getSize(void)
		{
			return _buffSize;
		}

		/**  Get the current read position within the buffer
		*/
		uint16_t getReadIndex(void)
		{
			return _readIndex;
		}

		/**  Get the current write position within the buffer
		*/
		uint16_t getWriteIndex(void)
		{
			return _writeIndex;
		}

		/**  Reset the maximum & minimum values
		*/
		void resetMaxMin(void)
		{
			_min = (sizeof(buff_type) * UINT8_MAX);
			_max = 0;
		}

		/** Destroy the buffer by freeing the memory
		*/
		void destroy(void)						
		{
			free(_buff);

			_buffSize = 0;
			_total = 0;
			_mean = 0;
			_min = 0;
			_max = 0;

			_readIndex = 0;
			_writeIndex = 0;

			_buff = NULL;
		}

	private:
		uint16_t _buffSize;		/**< size of the buffer (must be a power of 2) */
		uint16_t _buffSizeMask;	/**< mask used to calculate the wrap around point */
		buff_type *_buff;		/**< data buffer */

		uint16_t _readIndex;	/**< index to read data from */
		uint16_t _writeIndex;	/**< index to write new data to */

		long double _total;		/**< running total */
		buff_type _mean;		/**< running mean */
		buff_type _min;			/**< all-time minimum value */
		buff_type _max;			/**< all-time maximum value */

		
		bool isPowerOfTwo(uint16_t x)			/**< return true if 'x' is a power of 2 */		
		{
			return (((uint16_t)x & ((uint16_t)x - (uint16_t)1)) == (uint16_t)0);
		}
};


#endif // CIRCLE_BUFFER_H_