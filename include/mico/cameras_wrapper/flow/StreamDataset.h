//---------------------------------------------------------------------------------------------------------------------
//  Cameras wrapper MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------



#ifndef MICO_FLOW_BLOCKS_STREAMERS_STREAMDATASET_H_
#define MICO_FLOW_BLOCKS_STREAMERS_STREAMDATASET_H_

#include <flow/Block.h>
#include <mico/cameras_wrapper/StereoCameras/StereoCameraVirtual.h>

namespace mico{

    class StreamDataset:public flow::Block{
    public:
        virtual std::string name() const override {return "Dataset Streamer";}
        
        StreamDataset();
        // ~StreamDataset(){};
        
        virtual bool configure(std::unordered_map<std::string, std::string> _params) override;
        std::vector<std::string> parameters() override;

        std::string description() const override {return    "Streamer block that reads data from a dataset (in split files format) and streams it syncronously."
                                                            "It assumes that all the files are sequentially indexed.\n"
                                                            "   - Outputs: \n";};
        

        virtual QWidget * customWidget() override;

    protected:
        virtual void loopCallback() override;

    private:
        StereoCameraVirtual camera_;
        float targetRate_ = 30; // FPS
    };

}



#endif