class FoobarPipeline extends OpenCvPipeline
{
    int lastResult = 0;

    @Override
    public Mat processFrame(Mat input)
    {
        // ... some image processing here ...

        if(...)
        {
            lastResult = 1;
        }
        else if(...)
        {
            lastResult = 2
        }
        else if(...)
        {
            lastResult = 3;
        }
    }

    public int getLatestResults()
    {
        return lastResult;
    }
}