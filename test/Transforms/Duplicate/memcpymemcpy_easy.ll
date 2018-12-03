; ModuleID = 'memcpymemcpy_easy.c'
source_filename = "memcpymemcpy_easy.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

@.str = private unnamed_addr constant [3 x i8] c"%d\00", align 1
@.str.1 = private unnamed_addr constant [4 x i8] c"%d\0A\00", align 1

; Function Attrs: noinline nounwind uwtable
define dso_local void @f(i32* nocapture readonly %a, i32* nocapture %b, i32 %i) local_unnamed_addr #0 {
entry:
  %cmp = icmp sgt i32 %i, 0
  %0 = bitcast i32* %b to i8*
  %1 = bitcast i32* %a to i8*
  br i1 %cmp, label %if.then, label %if.else

if.then:                                          ; preds = %entry
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 4 %0, i8* align 4 %1, i64 24, i1 false)
  br label %if.end

if.else:                                          ; preds = %entry
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 4 %0, i8* align 4 %1, i64 32, i1 false)
  br label %if.end

if.end:                                           ; preds = %if.else, %if.then
  tail call void @llvm.memcpy.p0i8.p0i8.i64(i8* nonnull align 4 %0, i8* nonnull align 4 %1, i64 32, i1 false)
  ret void
}

; Function Attrs: argmemonly nounwind
declare void @llvm.memcpy.p0i8.p0i8.i64(i8* nocapture writeonly, i8* nocapture readonly, i64, i1) #1

; Function Attrs: noinline nounwind uwtable
define dso_local i32 @main() local_unnamed_addr #0 {
entry:
  %i = alloca i32, align 4
  %0 = bitcast i32* %i to i8*
  call void @llvm.lifetime.start.p0i8(i64 4, i8* nonnull %0) #3
  %call = call i32 (i8*, ...) @__isoc99_scanf(i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str, i64 0, i64 0), i32* nonnull %i)
  %call1 = call noalias i8* @malloc(i64 4096) #3
  %1 = bitcast i8* %call1 to i32*
  %call2 = call noalias i8* @malloc(i64 4096) #3
  %2 = bitcast i8* %call2 to i32*
  %3 = load i32, i32* %i, align 4, !tbaa !2
  %arrayidx = getelementptr inbounds i8, i8* %call1, i64 8
  %4 = bitcast i8* %arrayidx to i32*
  store i32 %3, i32* %4, align 4, !tbaa !2
  %arrayidx3 = getelementptr inbounds i8, i8* %call1, i64 16
  %5 = bitcast i8* %arrayidx3 to i32*
  store i32 %3, i32* %5, align 4, !tbaa !2
  call void @f(i32* %1, i32* %2, i32 %3)
  %arrayidx4 = getelementptr inbounds i8, i8* %call2, i64 8
  %6 = bitcast i8* %arrayidx4 to i32*
  %7 = load i32, i32* %6, align 4, !tbaa !2
  %call5 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([4 x i8], [4 x i8]* @.str.1, i64 0, i64 0), i32 %7)
  %call7 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([4 x i8], [4 x i8]* @.str.1, i64 0, i64 0), i32 %3)
  call void @llvm.lifetime.end.p0i8(i64 4, i8* nonnull %0) #3
  ret i32 0
}

; Function Attrs: argmemonly nounwind
declare void @llvm.lifetime.start.p0i8(i64, i8* nocapture) #1

; Function Attrs: nounwind
declare dso_local i32 @__isoc99_scanf(i8* nocapture readonly, ...) local_unnamed_addr #2

; Function Attrs: nounwind
declare dso_local noalias i8* @malloc(i64) local_unnamed_addr #2

; Function Attrs: nounwind
declare dso_local i32 @printf(i8* nocapture readonly, ...) local_unnamed_addr #2

; Function Attrs: argmemonly nounwind
declare void @llvm.lifetime.end.p0i8(i64, i8* nocapture) #1

attributes #0 = { noinline nounwind uwtable "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { argmemonly nounwind }
attributes #2 = { nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #3 = { nounwind }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 7.0.1 (https://github.com/llvm-mirror/clang 64485e37ac4b73bdf0b1834164dee90f7cae2eea) (git@github.com:eHammarstrom/llvm-dbds.git 21e32a934b255ad722d5992a657dd80fcbc40891)"}
!2 = !{!3, !3, i64 0}
!3 = !{!"int", !4, i64 0}
!4 = !{!"omnipotent char", !5, i64 0}
!5 = !{!"Simple C/C++ TBAA"}
