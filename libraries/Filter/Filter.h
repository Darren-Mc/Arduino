#ifndef Filter_h
#define Filter_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

template <class T> class Filter
{
	private:
		// Private functions and variables here.  They can only be accessed
        // by functions within the class.
		T *_store;
		T *_copy;
		T _sum;
		uint32_t _position;                                   // _position variable for circular buffer
        uint32_t _count;
        uint32_t _size;
		uint32_t _cutoff;									  // _cutoff is the percentage of values to be removed from each side when calculating mean
		void swap(T &a, T &b);
		int SplitArray(T* array, int pivot, int startIndex, int endIndex);
		void QuickSort(T* array, int startIndex, int endIndex);
	
	public:
		// Public functions and variables.  These can be accessed from
        // outside the class.
		Filter(uint32_t size);
		~Filter();
		void push(T entry);
		float mean();
		T get(uint32_t);
		int getCount();
		void sort();
		void printArray();
		void setCutOff(int cutoff);
};

template <class T> Filter<T>::Filter(uint32_t size) {
    _size = size;
    _count = 0;
    _store = (T *)malloc(sizeof(T) * size);
	_copy = (T *)malloc(sizeof(T) * size);
    _position = 0;                                            // track position for circular storage
    _sum = 0;                                                 // track sum for fast mean calculation
	_cutoff = 20;
    for (uint32_t i = 0; i < size; i++) {
        _store[i] = 0;
		_copy[i] = 0;
    }
}

template <class T> Filter<T>::~Filter() {
    free(_store);
}

template <class T> int Filter<T>::getCount() {
    return _count;
}

template <class T> void Filter<T>::setCutOff(int cutoff)
{
	if (cutoff < 50 && cutoff > 0)
	{
		_cutoff = cutoff;
	}
}

template <class T> void Filter<T>::push(T entry) {
    if (_count < _size) {                                     // adding new values to array
        _count++;                                             // count number of values in array
    } else {                                                    // overwriting old values
        _sum = _sum -_store[_position];                       // remove old value from _sum
    }
    _store[_position] = entry;                                // store new value in array
    _sum += entry;                                            // add the new value to _sum
    _position += 1;                                           // increment the position counter
    if (_position >= _size) _position = 0;                    // loop the position counter
}

template <class T> T Filter<T>::get(uint32_t index) {
    if (index >= _count) {
        return 0;
    }
    int32_t cindex = _position-index;                         // position in circular buffer
    if (cindex < 0) cindex = _size + cindex;                  // need to loop around for negative cindex
    return _store[cindex];
}

template <class T> void Filter<T>::sort()
{
	for (int i = 0; i < _count; i++)
	{
		_copy[i] = _store[i];
	}
	QuickSort(_copy,0,_count-1);
	/*for (int i = 0; i < _count; i++)
	{
		Serial.print(_copy[i]);
		if (i + 1 < _count) Serial.print(",");
		else Serial.print("\n");
	}*/
	
	int cutoff = round((float)(_count*_cutoff)/100.0f);
	
	if (_count-2*cutoff == 0 && cutoff > 0)
	{
		cutoff -= 1;
	}
	
	/*for (int i = cutoff; i < _count - cutoff; i++)
	{
		Serial.print(_copy[i]);
		Serial.print(",");
	}
	Serial.print("\n");*/
}

template <class T> void Filter<T>::printArray()
{
	for (int i = 0; i < _count; i++)
	{
		Serial.print(_store[i]);
		if (i + 1 < _count) Serial.print(",");
		else Serial.print("\n");
	}
}

template <class T> float Filter<T>::mean()
{
	if (_count == 0) {
        return 0;
    }
	
	Filter::sort();
	
	float sum = 0;
	
	int cutoff = round((float)(_count*_cutoff)/100.0f);
	
	if (_count-2*cutoff == 0 && cutoff > 0)
	{
		cutoff -= 1;
	}
	
	for (int i = cutoff; i < _count - cutoff; i++)
	{
		sum += _copy[i];
	}
    return (sum / (float)(_count-2*cutoff));
}

/* This function swaps two numbers
   Arguments :
             a, b - the numbers to be swapped
   */
template <class T> void Filter<T>::swap(T &a, T &b)
{
    T temp;
    temp = a;
    a = b;
    b = temp;
}

/* This function performs a quicksort
   Arguments :
             array - the array to be sorted
             startIndex - index of the first element of the section
             endIndex - index of the last element of the section
   */
template <class T> void Filter<T>::QuickSort(T* array, int startIndex, int endIndex)
{
    int pivot = array[startIndex];                  //pivot element is the leftmost element
    int splitPoint;
     
    if(endIndex > startIndex)                         //if they are equal, it means there is
                                                      //only one element and quicksort's job
                                                      //here is finished
    {
        splitPoint = SplitArray(array, pivot, startIndex, endIndex);
                                                      //SplitArray() returns the position where
                                                      //pivot belongs to
        array[splitPoint] = pivot;
        QuickSort(array, startIndex, splitPoint-1);   //Quick sort first half
        QuickSort(array, splitPoint+1, endIndex);    //Quick sort second half
    }
}

/* This function splits the array around the pivot
   Arguments :
             array - the array to be split
             pivot - pivot element whose position will be returned
             startIndex - index of the first element of the section
             endIndex - index of the last element of the section
   Returns :
           the position of the pivot
   */
template <class T> int Filter<T>::SplitArray(T* array, int pivot, int startIndex, int endIndex)
{
    int leftBoundary = startIndex;
    int rightBoundary = endIndex;
     
    while(leftBoundary < rightBoundary)             //shuttle pivot until the boundaries meet
    {
         while( pivot < array[rightBoundary]       //keep moving until a lesser element is found
                && rightBoundary > leftBoundary)   //or until the leftBoundary is reached
         {
              rightBoundary--;                      //move left
         }
         swap(array[leftBoundary], array[rightBoundary]);
         //PrintArray(array, ARRAY_SIZE);            //Uncomment this line for study
          
         while( pivot >= array[leftBoundary]       //keep moving until a greater or equal element is found
                && leftBoundary < rightBoundary)   //or until the rightBoundary is reached
         {
              leftBoundary++;                        //move right
         }
         swap(array[rightBoundary], array[leftBoundary]);
         //PrintArray(array, ARRAY_SIZE);            //Uncomment this line for study
    }
    return leftBoundary;                              //leftBoundary is the split point because

                                                      //the above while loop exits only when
                                                      //leftBoundary and rightBoundary are equal
}

#endif