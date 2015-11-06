/* Copyright (c) 2013-2015 Stanislaw Halik <sthalik@misaki.pl>
 * Copyright (c) 2015 Wim Vriend
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#include "ftnoir_protocol_ft.h"
#include "csv/csv.h"

FTNoIR_Protocol::FTNoIR_Protocol() :
    shm(FREETRACK_HEAP, FREETRACK_MUTEX, sizeof(FTHeap)),
    pMemData((FTHeap*) shm.ptr()),
    viewsStart(nullptr),
    viewsStop(nullptr),
    intGameID(0)
{
}

FTNoIR_Protocol::~FTNoIR_Protocol()
{
    if (viewsStop != NULL) {
        viewsStop();
        FTIRViewsLib.unload();
    }
    dummyTrackIR.terminate();
    dummyTrackIR.kill();
    dummyTrackIR.waitForFinished(50);
}

void FTNoIR_Protocol::pose(const double* headpose) {
    float yaw = -getRadsFromDegrees(headpose[Yaw]);
    float pitch = -getRadsFromDegrees(headpose[Pitch]);
    float roll = getRadsFromDegrees(headpose[Roll]);
    float tx = headpose[TX] * 10.f;
    float ty = headpose[TY] * 10.f;
    float tz = headpose[TZ] * 10.f;
    
    FTHeap* ft = pMemData;
    FTData* data = &ft->data;

    data->RawX = 0;
    data->RawY = 0;
    data->RawZ = 0;
    data->RawPitch = 0;
    data->RawYaw = 0;
    data->RawRoll = 0;
    
    data->X = tx;
    data->Y = ty;
    data->Z = tz;
    data->Yaw = yaw;
    data->Pitch = pitch;
    data->Roll = roll;
    
    data->X1 = data->DataID;
    data->X2 = 0;
    data->X3 = 0;
    data->X4 = 0;
    data->Y1 = 0;
    data->Y2 = 0;
    data->Y3 = 0;
    data->Y4 = 0;
    
    int32_t id = ft->GameID;
    
    if (intGameID != id)
    {
        shm.lock();
        
        QString gamename;
        {
            unsigned char table[8];
            if (CSV::getGameData(id, table, gamename))
                for (int i = 0; i < 8; i++) pMemData->table[i] = table[i];
        }
        ft->GameID2 = id;
        intGameID = id;
        QMutexLocker foo(&this->game_name_mutex);
        connected_game = gamename;
        
        shm.unlock();
    }
    
    data->DataID += 1;
}

void FTNoIR_Protocol::start_tirviews() {
    QString aFileName = QCoreApplication::applicationDirPath() + "/TIRViews.dll";
    if ( QFile::exists( aFileName )) {
        FTIRViewsLib.setFileName(aFileName);
        FTIRViewsLib.load();

        viewsStart = (importTIRViewsStart) FTIRViewsLib.resolve("TIRViewsStart");
        if (viewsStart == NULL) {
            qDebug() << "FTServer::run() says: TIRViewsStart function not found in DLL!";
        }
        else {
            qDebug() << "FTServer::run() says: TIRViewsStart executed!";
            viewsStart();
        }

        //
        // Load the Stop function from TIRViews.dll. Call it when terminating the thread.
        //
        viewsStop = (importTIRViewsStop) FTIRViewsLib.resolve("TIRViewsStop");
        if (viewsStop == NULL) {
            qDebug() << "FTServer::run() says: TIRViewsStop function not found in DLL!";
        }
    }
}

void FTNoIR_Protocol::start_dummy() {


    QString program = QCoreApplication::applicationDirPath() + "/TrackIR.exe";
    dummyTrackIR.setProgram("\"" + program + "\"");
    dummyTrackIR.start();
}

bool FTNoIR_Protocol::correct()
{
    // Registry settings (in HK_USER)
    QSettings settings("Freetrack", "FreetrackClient");
    QSettings settingsTIR("NaturalPoint", "NATURALPOINT\\NPClient Location");

    if (!shm.success())
        return false;

    QString aLocation =  QCoreApplication::applicationDirPath() + "/";

    switch (s.intUsedInterface) {
    case 0:
        // Use both interfaces
        settings.setValue( "Path" , aLocation );
        settingsTIR.setValue( "Path" , aLocation );
        break;
    case 1:
        // Use FreeTrack, disable TrackIR
        settings.setValue( "Path" , aLocation );
        settingsTIR.setValue( "Path" , "" );
        break;
    case 2:
        // Use TrackIR, disable FreeTrack
        settings.setValue( "Path" , "" );
        settingsTIR.setValue( "Path" , aLocation );
        break;
    default:
        break;
    }

    if (s.useTIRViews) {
        start_tirviews();
    }

    // more games need the dummy executable than previously thought
    start_dummy();

    pMemData->data.DataID = 1;
    pMemData->data.CamWidth = 100;
    pMemData->data.CamHeight = 250;
    pMemData->GameID2 = 0;
    for (int i = 0; i < 8; i++)
        pMemData->table[i] = 0;

    return true;
}

OPENTRACK_DECLARE_PROTOCOL(FTNoIR_Protocol, FTControls, FTNoIR_ProtocolDll)
