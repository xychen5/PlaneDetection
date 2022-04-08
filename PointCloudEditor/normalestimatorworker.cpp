#include "time.h"

#include "normalestimatorworker.h"

//#include <tbb/parallel_for.h>
//#include <tbb/blocked_range.h>
//#include <tbb/mutex.h>

#include "boundaryvolumehierarchy.h"
#include "ThreadPool.h"

NormalEstimatorWorker::NormalEstimatorWorker(PointCloud3d *pointCloud)
    : mPointCloud(pointCloud)
    , mNumNeighbors(21)
    , mSpeed(NormalEstimator3d::QUICK)
{

}

void NormalEstimatorWorker::actions()
{
    clock_t cur, last;
    last = clock();

    if (mPointCloud == NULL) return;

    emit workerStatus("Pre-processing...");

    Octree octree(mPointCloud);
    octree.partition(10, 30);

    emit workerStatus("Estimating normals...");

    ConnectivityGraph *connectivity = new ConnectivityGraph(mPointCloud->size());
    mPointCloud->connectivity(connectivity);

    NormalEstimator3d estimator(&octree, mNumNeighbors, mSpeed);

    cur = clock();
    last = cur;
    std::cout << "[estimate normals / octree & connectGraph]: Done in " + std::to_string((double)(cur - last)/CLOCKS_PER_SEC) + "s." << std::endl;

    //tbb::mutex mutex;
    size_t count = 0;
    ThreadPool pool(std::thread::hardware_concurrency());
    pool.init();

    size_t eachThreadDealPointNum = mPointCloud->size() / std::thread::hardware_concurrency() + 1;

    std::mutex l;
    auto calNormal = [&](size_t st, size_t ed) {
        std::cout << "do calNormal: " << st << " -> " << ed << std::endl;
        for (size_t i = st; i < std::min(mPointCloud->size(), ed); i++)
        {
            // std::cout << i << " max: " << std::min(mPointCloud->size(), ed) << std::endl;
            if (this->isRunning())
            {
                NormalEstimator3d::Normal normal = estimator.estimate(i);

                // because the connectivity's increasing must be sequential
                // so we shall at first tail apart the pointcloud into serveral part
                // and do the normal estimation
                l.lock();
                connectivity->addNode(i, normal.neighbors);
                l.unlock();

                (*mPointCloud)[i].normal(normal.normal);
                (*mPointCloud)[i].normalConfidence(normal.confidence);
                (*mPointCloud)[i].curvature(normal.curvature);
                // std::lock_guard<std::mutex> lock(l);
                ++count;
                if (count % 1000 == 0)
                {
                    std::cout << count << std::endl;
                    emit workerProgress(static_cast<float>(count) / mPointCloud->size());
                }
            }
        }
    };

    // start thread to parallel
    for(size_t seg = 0; seg < std::thread::hardware_concurrency(); ++seg) {
        std::cout << "submit thread: " << seg << std::endl;
        pool.submit(calNormal, seg * eachThreadDealPointNum, (1 + seg) * (eachThreadDealPointNum));
    }
    pool.shutdown();

    //tbb::parallel_for(tbb::blocked_range<size_t>(0, mPointCloud->size()), [&](const tbb::blocked_range<size_t> &x) {
    //    for (size_t i = x.begin(); i != x.end(); ++i)
    //    for (size_t i = 0; i < mPointCloud->size(); i++)
    //    {
    //        if (isRunning())
    //        {
    //            NormalEstimator3d::Normal normal = estimator.estimate(i);
    //            connectivity->addNode(i, normal.neighbors);
    //            (*mPointCloud)[i].normal(normal.normal);
    //            (*mPointCloud)[i].normalConfidence(normal.confidence);
    //            (*mPointCloud)[i].curvature(normal.curvature);
    //            //mutex.lock();
    //            ++count;
    //            if (count % 1000 == 0)
    //            {
    //                emit workerProgress(static_cast<float>(count) / mPointCloud->size());
    //            }
    //            //mutex.unlock();
    //        }
    //    }
    // });
    cur = clock();
    std::cout << "[estimate normals / all]: Done in " + std::to_string((double)(cur - last)/CLOCKS_PER_SEC) + "s." << std::endl;
}
