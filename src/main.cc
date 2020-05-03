#include <node.h>
#include <nan.h>
#include <stdio.h>

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

using namespace v8;

static float frand() {
    return (float)rand()/(float)RAND_MAX;
}

class NavQuery : public Nan::ObjectWrap {
private:
	dtNavMesh *m_navMesh;
	dtNavMeshQuery *m_navQuery;

	NavQuery() {
		m_navMesh = dtAllocNavMesh();
		m_navQuery = dtAllocNavMeshQuery();
	}
	~NavQuery() {
		dtFreeNavMesh(m_navMesh);
		m_navMesh = NULL;
		dtFreeNavMeshQuery(m_navQuery);
		m_navQuery = NULL;
	}
public:
	static NAN_MODULE_INIT(Init) {
        srand(time(0));

		v8::Local<v8::FunctionTemplate> prototype = Nan::New<v8::FunctionTemplate>(New);
		prototype->SetClassName(Nan::New("NavQuery").ToLocalChecked());
		prototype->InstanceTemplate()->SetInternalFieldCount(1);

		Nan::SetPrototypeMethod(prototype, "load", Load);
        Nan::SetPrototypeMethod(prototype, "clear", Clear);
		Nan::SetPrototypeMethod(prototype, "findNearestPoly", FindNearestPoly);
        Nan::SetPrototypeMethod(prototype, "findRandomPoint", FindRandomPoint);
		Nan::SetPrototypeMethod(prototype, "findStraightPath", FindStraightPath);

		constructor().Reset(Nan::GetFunction(prototype).ToLocalChecked());
		Nan::Set(target, Nan::New("NavQuery").ToLocalChecked(), Nan::GetFunction(prototype).ToLocalChecked());
	}

	static NAN_METHOD(New) {
		if (info.IsConstructCall()) {
			NavQuery *thisObject = new NavQuery();
			thisObject->Wrap(info.This());
			info.GetReturnValue().Set(info.This());
		}
	}

	static NAN_METHOD(FindNearestPoly) {
		NavQuery* thisObject = Nan::ObjectWrap::Unwrap<NavQuery>(info.Holder());
		const float center[] = {
			(float)Nan::To<double>(info[0]).FromJust(),
			(float)Nan::To<double>(info[1]).FromJust(),
			(float)Nan::To<double>(info[2]).FromJust(),
		};
		const float halfExtents[] = {
			(float)Nan::To<double>(info[3]).FromJust(),
			(float)Nan::To<double>(info[4]).FromJust(),
			(float)Nan::To<double>(info[5]).FromJust(),
		};
		dtQueryFilter filter;
		filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL);
		filter.setExcludeFlags(0);
		float nearestPt[3];
		dtPolyRef nearestRef = 0;
        dtStatus status = 0;
		status = thisObject->m_navQuery->findNearestPoly(center, halfExtents, &filter, &nearestRef, nearestPt);
        if (dtStatusFailed(status)) {
            info.GetReturnValue().Set(Nan::New(status));
            return;
        }
		if (!nearestRef) {
            info.GetReturnValue().Set(Nan::Null());
            return;
		}
		v8::Local<v8::Object> result = Nan::New<v8::Object>();
	    Nan::Set(result, Nan::New("x").ToLocalChecked(), Nan::New(nearestPt[0]));
	    Nan::Set(result, Nan::New("y").ToLocalChecked(), Nan::New(nearestPt[1]));
	    Nan::Set(result, Nan::New("z").ToLocalChecked(), Nan::New(nearestPt[2]));
	    Nan::Set(result, Nan::New("ref").ToLocalChecked(), Nan::New(nearestRef));
	    info.GetReturnValue().Set(result);
	}
    
    static NAN_METHOD(FindRandomPoint) {
        NavQuery* thisObject = Nan::ObjectWrap::Unwrap<NavQuery>(info.Holder());
        dtQueryFilter filter;
        dtPolyRef randomRef;
        float randomPt[3];
        dtStatus status = 0;
        status = thisObject->m_navQuery->findRandomPoint(&filter, frand, &randomRef, randomPt);
        if (dtStatusFailed(status)) {
            info.GetReturnValue().Set(Nan::New(status));
            return;
        }
        v8::Local<v8::Object> result = Nan::New<v8::Object>();
        Nan::Set(result, Nan::New("x").ToLocalChecked(), Nan::New(randomPt[0]));
        Nan::Set(result, Nan::New("y").ToLocalChecked(), Nan::New(randomPt[1]));
        Nan::Set(result, Nan::New("z").ToLocalChecked(), Nan::New(randomPt[2]));
        Nan::Set(result, Nan::New("ref").ToLocalChecked(), Nan::New(randomRef));
        info.GetReturnValue().Set(result);
    }
    
	static NAN_METHOD(FindStraightPath) {
		NavQuery* thisObject = Nan::ObjectWrap::Unwrap<NavQuery>(info.Holder());
        if (!info[0]->IsObject() || !info[1]->IsObject()) {
            info.GetReturnValue().Set(Nan::New(0));
            return;
        }
        v8::Local<v8::Object> startObject = Nan::To<v8::Object>(info[0]).ToLocalChecked();
        v8::Local<v8::Object> endObject = Nan::To<v8::Object>(info[1]).ToLocalChecked();
        dtPolyRef startRef = (int)Nan::To<int>(Nan::Get(startObject, Nan::New("ref").ToLocalChecked()).ToLocalChecked()).FromJust();
        dtPolyRef endRef = (int)Nan::To<int>(Nan::Get(endObject, Nan::New("ref").ToLocalChecked()).ToLocalChecked()).FromJust();
		const float startPos[] = {
            (float)Nan::To<double>(Nan::Get(startObject, Nan::New("x").ToLocalChecked()).ToLocalChecked()).FromJust(),
            (float)Nan::To<double>(Nan::Get(startObject, Nan::New("y").ToLocalChecked()).ToLocalChecked()).FromJust(),
            (float)Nan::To<double>(Nan::Get(startObject, Nan::New("z").ToLocalChecked()).ToLocalChecked()).FromJust(),
        };
		const float endPos[] = {
            (float)Nan::To<double>(Nan::Get(endObject, Nan::New("x").ToLocalChecked()).ToLocalChecked()).FromJust(),
            (float)Nan::To<double>(Nan::Get(endObject, Nan::New("y").ToLocalChecked()).ToLocalChecked()).FromJust(),
            (float)Nan::To<double>(Nan::Get(endObject, Nan::New("z").ToLocalChecked()).ToLocalChecked()).FromJust(),
        };
		dtQueryFilter filter;
        const int maxPath = 2048;
		dtPolyRef path[maxPath];
		int pathCount = 0;
        float straightPath[maxPath * 3];
        unsigned char straightPathFlags[maxPath];
        dtPolyRef straightPathRefs[maxPath];
        int straightPathCount = 0;
        dtStatus status = 0;
        status = thisObject->m_navQuery->findPath(startRef, endRef, startPos, endPos, &filter, path, &pathCount, maxPath);
        if (dtStatusFailed(status)) {
            info.GetReturnValue().Set(Nan::New(status));
            return;
        }
        status = thisObject->m_navQuery->findStraightPath(startPos, endPos, path, pathCount, straightPath, straightPathFlags, straightPathRefs, &straightPathCount, maxPath, 0);
        if (dtStatusFailed(status)) {
            info.GetReturnValue().Set(Nan::New(status));
            return;
        }
        v8::Local<v8::Array> result = Nan::New<v8::Array>(straightPathCount);
        for (int index = 0; index < straightPathCount; index++) {
            v8::Local<v8::Object> vector3 = Nan::New<v8::Object>();
            const int cursor = index * 3;
            Nan::Set(vector3, Nan::New("x").ToLocalChecked(), Nan::New(straightPath[cursor + 0]));
            Nan::Set(vector3, Nan::New("y").ToLocalChecked(), Nan::New(straightPath[cursor + 1]));
            Nan::Set(vector3, Nan::New("z").ToLocalChecked(), Nan::New(straightPath[cursor + 2]));
            Nan::Set(vector3, Nan::New("ref").ToLocalChecked(), Nan::New(straightPathRefs[index]));
            Nan::Set(vector3, Nan::New("flags").ToLocalChecked(), Nan::New(straightPathFlags[index]));
            Nan::Set(result, index, vector3);
        }
        info.GetReturnValue().Set(result);
	}

    static NAN_METHOD(Clear) {
        NavQuery* thisObject = Nan::ObjectWrap::Unwrap<NavQuery>(info.Holder());
        dtFreeNavMesh(thisObject->m_navMesh);
        thisObject->m_navMesh = dtAllocNavMesh();
        thisObject->m_navQuery->init(thisObject->m_navMesh, 2048);
        info.GetReturnValue().Set(Nan::True());
    }
        
	static NAN_METHOD(Load) {
		Isolate *isolate = info.GetIsolate();
		NavQuery* thisObject = Nan::ObjectWrap::Unwrap<NavQuery>(info.Holder());
		if (info[0]->IsString() == false) {
			isolate->ThrowException(Nan::Error("The \"path\" argument must be of type string"));
			return;
		}
		char charBuffer[1024];
		dtStatus status = 0;
		dtNavMesh *navMesh = NULL;
		Nan::Utf8String path(info[0]);

		Nan::Set(info.This(), Nan::New("filename").ToLocalChecked(), info[0]);
		
		Sample sample;
		navMesh = sample.loadAll(*path);
		if (navMesh) {
			dtFreeNavMesh(thisObject->m_navMesh);
			thisObject->m_navMesh = navMesh;
			thisObject->m_navQuery->init(thisObject->m_navMesh, 2048);
			info.GetReturnValue().Set(Nan::True());
			return;
		}

		FILE *fp = fopen(*path, "rb");
		if (!fp) {
			isolate->ThrowException(Nan::Error("No such file or directory"));
			return;
		}
		if (fseek(fp, 0, SEEK_END) != 0) {
			fclose(fp);
			isolate->ThrowException(Nan::Error("fseek"));
			return;
		}
		long bufferSize = ftell(fp);
		if (bufferSize < 0) {
			fclose(fp);
			isolate->ThrowException(Nan::Error("ftell"));
			return;
		}
		if (fseek(fp, 0, SEEK_SET) != 0) {
			fclose(fp);
			isolate->ThrowException(Nan::Error("fseek"));
			return;
		}
		unsigned char *buffer = (unsigned char*)dtAlloc(bufferSize, DT_ALLOC_PERM);
		if (!buffer) {
			fclose(fp);
			isolate->ThrowException(Nan::Error("Out of Memory"));
			return;
		}
		size_t readSize = fread(buffer, bufferSize, 1, fp);
		fclose(fp);
		if (readSize != 1) {
            dtFree(buffer);
			isolate->ThrowException(Nan::Error("fread"));
			return;
		}
		navMesh = dtAllocNavMesh();
		if (!navMesh) {
            dtFree(buffer);
			isolate->ThrowException(Nan::Error("dtAllocNavMesh"));
			return;
		}
		status = navMesh->init(buffer, bufferSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status)) {
            dtFree(buffer);
			dtFreeNavMesh(navMesh);
			sprintf(charBuffer, "dtNavMesh->init 0x%x", status);
			isolate->ThrowException(Nan::Error(charBuffer));
			return;
		}
		dtFreeNavMesh(thisObject->m_navMesh);
        thisObject->m_navMesh = navMesh;
        thisObject->m_navQuery->init(thisObject->m_navMesh, 2048);
        info.GetReturnValue().Set(Nan::True());
	}

	static inline Nan::Persistent<v8::Function> & constructor() {
		static Nan::Persistent<v8::Function> my_constructor;
		return my_constructor;
	}
};

NODE_MODULE(NODE_GYP_MODULE_NAME, NavQuery::Init)
