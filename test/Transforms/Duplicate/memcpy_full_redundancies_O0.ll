; ModuleID = 'redundant_memcpy3.c'
source_filename = "redundant_memcpy3.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

@.str = private unnamed_addr constant [4 x i8] c"%c\0A\00", align 1

; Function Attrs: noinline nounwind optnone uwtable
define dso_local void @f(i32* %a, i32* %b, i32 %i) #0 {
entry:
  %a.addr = alloca i32*, align 8
  %b.addr = alloca i32*, align 8
  %i.addr = alloca i32, align 4
  store i32* %a, i32** %a.addr, align 8
  store i32* %b, i32** %b.addr, align 8
  store i32 %i, i32* %i.addr, align 4
  %0 = load i32, i32* %i.addr, align 4
  %cmp = icmp sgt i32 %0, 0
  br i1 %cmp, label %if.then, label %if.else

if.then:                                          ; preds = %entry
  %1 = load i32*, i32** %a.addr, align 8
  %2 = bitcast i32* %1 to i8*
  %3 = load i32*, i32** %b.addr, align 8
  %4 = bitcast i32* %3 to i8*
  call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 4 %2, i8* align 4 %4, i64 16, i1 false)
  br label %if.end

if.else:                                          ; preds = %entry
  %5 = load i32*, i32** %a.addr, align 8
  %6 = bitcast i32* %5 to i8*
  %7 = load i32*, i32** %b.addr, align 8
  %8 = bitcast i32* %7 to i8*
  call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 4 %6, i8* align 4 %8, i64 32, i1 false)
  %9 = load i32*, i32** %a.addr, align 8
  %10 = bitcast i32* %9 to i8*
  %11 = load i32*, i32** %b.addr, align 8
  %12 = bitcast i32* %11 to i8*
  call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 4 %10, i8* align 4 %12, i64 32, i1 false)
  br label %if.end

if.end:                                           ; preds = %if.else, %if.then
  %13 = load i32*, i32** %a.addr, align 8
  %14 = bitcast i32* %13 to i8*
  %15 = load i32*, i32** %b.addr, align 8
  %16 = bitcast i32* %15 to i8*
  call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 4 %14, i8* align 4 %16, i64 32, i1 false)
  ret void
}

; Function Attrs: argmemonly nounwind
declare void @llvm.memcpy.p0i8.p0i8.i64(i8* nocapture writeonly, i8* nocapture readonly, i64, i1) #1

; Function Attrs: noinline nounwind optnone uwtable
define dso_local i32 @main() #0 {
entry:
  %retval = alloca i32, align 4
  %a = alloca i32*, align 8
  %b = alloca i32*, align 8
  %i = alloca i32, align 4
  store i32 0, i32* %retval, align 4
  %call = call i32 (...) @get()
  store i32 %call, i32* %i, align 4
  %call1 = call noalias i8* @malloc(i64 4096) #4
  %0 = bitcast i8* %call1 to i32*
  store i32* %0, i32** %a, align 8
  %call2 = call noalias i8* @malloc(i64 4096) #4
  %1 = bitcast i8* %call2 to i32*
  store i32* %1, i32** %b, align 8
  %2 = load i32*, i32** %a, align 8
  %3 = load i32*, i32** %b, align 8
  %4 = load i32, i32* %i, align 4
  call void @f(i32* %2, i32* %3, i32 %4)
  %5 = load i32*, i32** %b, align 8
  %arrayidx = getelementptr inbounds i32, i32* %5, i64 2
  %6 = load i32, i32* %arrayidx, align 4
  %call3 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([4 x i8], [4 x i8]* @.str, i32 0, i32 0), i32 %6)
  %7 = load i32*, i32** %a, align 8
  %arrayidx4 = getelementptr inbounds i32, i32* %7, i64 4
  %8 = load i32, i32* %arrayidx4, align 4
  %call5 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([4 x i8], [4 x i8]* @.str, i32 0, i32 0), i32 %8)
  ret i32 0
}

declare dso_local i32 @get(...) #2

; Function Attrs: nounwind
declare dso_local noalias i8* @malloc(i64) #3

declare dso_local i32 @printf(i8*, ...) #2

attributes #0 = { noinline nounwind uwtable "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="true" "no-frame-pointer-elim-non-leaf" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { argmemonly nounwind }
attributes #2 = { "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="true" "no-frame-pointer-elim-non-leaf" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #3 = { nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="true" "no-frame-pointer-elim-non-leaf" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #4 = { nounwind }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 7.0.1 (https://github.com/llvm-mirror/clang 65f84326edb6105fb0263f0b023719b491f8cf1a) (git@github.com:eHammarstrom/llvm-dbds.git af59d9afe10cd19c3b5f4c053bea5377c36a3e56)"}
