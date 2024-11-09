

class Singleton
{
    public:
        Singleton(Singleton &other) = delete;
        void operator=(const Singleton &) = delete;

        static Singleton* GetInstance();
    protected:
        static Singleton* singleton_;
        Singleton();
};
    