; ModuleID = 'redundant_memcpy3.c'
source_filename = "redundant_memcpy3.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-pc-linux-gnu"

@.str = private unnamed_addr constant [4 x i8] c"%c\0A\00", align 1

; Function Attrs: nounwind uwtable
define void @f(i32* nocapture, i32* nocapture readonly, i32) local_unnamed_addr #0 {
  %4 = icmp sgt i32 %2, 0
  %5 = bitcast i32* %0 to i8*
  %6 = bitcast i32* %1 to i8*
  br i1 %4, label %7, label %8

; <label>:7:                                      ; preds = %3
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* %5, i8* %6, i64 16, i32 4, i1 false)
  br label %9

; <label>:8:                                      ; preds = %3
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* %5, i8* %6, i64 32, i32 4, i1 false)
  br label %9

; <label>:9:                                      ; preds = %8, %7
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* nonnull %5, i8* nonnull %6, i64 32, i32 4, i1 false)
  ret void
}

; Function Attrs: argmemonly nounwind
declare void @llvm.memcpy.p0i8.p0i8.i64(i8* nocapture writeonly, i8* nocapture readonly, i64, i32, i1) #1

; Function Attrs: nounwind uwtable
define i32 @main() local_unnamed_addr #0 {
  %1 = tail call i32 (...) @get() #4
  %2 = tail call noalias i8* @malloc(i64 4096) #4
  %3 = tail call noalias i8* @malloc(i64 4096) #4
  %4 = icmp sgt i32 %1, 0
  br i1 %4, label %5, label %6

; <label>:5:                                      ; preds = %0
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* %2, i8* %3, i64 16, i32 4, i1 false) #4
  br label %7

; <label>:6:                                      ; preds = %0
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* %2, i8* %3, i64 32, i32 4, i1 false) #4
  br label %7

; <label>:7:                                      ; preds = %5, %6
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* nonnull %2, i8* nonnull %3, i64 32, i32 4, i1 false) #4
  %8 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([4 x i8], [4 x i8]* @.str, i64 0, i64 0), i32 undef)
  %9 = getelementptr inbounds i8, i8* %2, i64 16
  %10 = bitcast i8* %9 to i32*
  %11 = load i32, i32* %10, align 4, !tbaa !2
  %12 = tail call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([4 x i8], [4 x i8]* @.str, i64 0, i64 0), i32 %11)
  ret i32 0
}

declare i32 @get(...) local_unnamed_addr #2

; Function Attrs: nounwind
declare noalias i8* @malloc(i64) local_unnamed_addr #3

; Function Attrs: nounwind
declare i32 @printf(i8* nocapture readonly, ...) local_unnamed_addr #3

attributes #0 = { nounwind uwtable "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { argmemonly nounwind }
attributes #2 = { "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #3 = { nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #4 = { nounwind }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 6.0.0-1ubuntu2 (tags/RELEASE_600/final)"}
!2 = !{!3, !3, i64 0}
!3 = !{!"int", !4, i64 0}
!4 = !{!"omnipotent char", !5, i64 0}
!5 = !{!"Simple C/C++ TBAA"}
