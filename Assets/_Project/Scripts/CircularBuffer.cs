namespace Kart {
    public class CircularBuffer<T> {
        T[] buffer;
        int bufferSize;
        
        public CircularBuffer(int bufferSize) {
            this.bufferSize = bufferSize;
            buffer = new T[bufferSize];
        }
        
        public void Add(T item, int index) => buffer[index % bufferSize] = item;
        public T Get(int index) => buffer[index % bufferSize];
        public void Clear() => buffer = new T[bufferSize];
    }
}